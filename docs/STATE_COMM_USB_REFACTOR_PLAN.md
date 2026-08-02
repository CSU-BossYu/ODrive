# ODrive 状态机、通信与 USB 可观测性重构计划

## 1. 文档目的

本计划用于把当前工程从“状态、错误、通信和调试互相耦合”的结构，逐步改造成：

1. 单一所有者的安全状态机。
2. 与状态机正交的操作、就绪度和故障模型。
3. 精简、稳定的产品 CAN 协议。
4. 由 USB 承担的诊断、波形、校准和测试协议。
5. 可在 PC 本机运行、可由 Python 驱动的控制核心和状态机测试。

重构必须增量进行。任何阶段都必须保持功率级安全，并且不得用新的延时、`volatile bool`
握手或 CAN 调试字段掩盖失败。

## 2. 开始前的重要事实

### 2.1 当前工作区不是干净基线

开始执行前必须运行并保存：

```powershell
git status --short
git diff --stat
git diff -- Firmware/MotorControl Firmware/communication Firmware/Tests foc_ui
```

当前已知存在尚未提交的实验修改，涉及：

- `Firmware/MotorControl/axis.cpp`
- `Firmware/MotorControl/axis.hpp`
- `Firmware/MotorControl/encoder.cpp`
- `Firmware/MotorControl/encoder.hpp`
- `Firmware/MotorControl/main.cpp`
- `Firmware/MotorControl/motor.cpp`
- 若干 Python 测试

其中 `closed_loop_phase_feedback_ready_`、`closed_loop_controller_ready_` 和相关等待逻辑尚未完成
编译及硬件验证，不能视为稳定设计。不得覆盖、删除或悄悄吸收这些修改。应先区分：

1. 已经硬件证明必要的修复。
2. 仅用于诊断的脚本改进。
3. 未验证的临时握手补丁。

不得删除用户生成的 `dual_encoder_motion.csv`。

### 2.2 外部参考边界

- 达妙协议参考位于 `docs/reference/damiao/`。只借鉴其高频运行协议精简、固定反馈帧等原则。
- `F:\project_source_code\SguanFOC_Library` 只借鉴 High/Low/Main 循环分层和硬件适配接口。
- 不复制 SguanFOC 将生命周期、运动趋势、安全和故障塞入一个 `status` 的设计。

## 3. 目标架构

```text
CAN Transport -----\
                    > CommandService -> SafetySupervisor -> RealtimeMailbox -> 10 kHz ISR
USB Test Control --/

10 kHz ISR -> FeedbackSnapshot -> SafetySupervisor
10 kHz ISR -> CriticalEventRing ----\
10 kHz ISR -> ScopeRing -------------> USB Debug Task -> USB CDC -> Python Backend -> UI
Fault/Crash -> FaultStore -----------/
```

### 3.1 四个正交状态对象

```text
SafetyState:
  BOOT / SAFE_OFF / PREPARING / READY / ARMED / STOPPING / FAULT_LATCHED

Operation:
  NONE / CLOSED_LOOP / CALIBRATION / SELF_TEST

ReadinessSnapshot:
  epoch, feedback_sequence, encoder_ready, phase_ready,
  current_ready, controller_ready, power_stage_ready

FaultStore:
  first_fault, active_faults, consequence_chain, occurrence_count
```

禁止通过枚举值大小判断权限，例如 `state >= IDLE`。每项权限必须由显式谓词决定：

```cpp
can_run_controller(state, operation)
can_enable_pwm(state)
can_modify_config(state)
```

### 3.2 唯一所有权

- 只有 `SafetySupervisor` 可以请求 arm、正常 disarm、状态迁移和清除锁存故障。
- ISR 可以执行立即硬件关断，并提交故障事件；ISR 不负责拼装上层状态。
- CAN/USB callback 只解码并投递命令，不能直接修改 Axis、Controller、Motor 或配置。
- Encoder、Motor、Controller 只能报告带来源的事件，不能直接写父级错误位。

## 4. 不可破坏的系统约束

1. ISR 中禁止 USB/CAN 发送、`printf`、格式化、动态内存、阻塞和任务队列等待。
2. USB 未连接、阻塞或缓冲区溢出不得影响控制循环。
3. 波形丢失不能挤掉关键故障事件。
4. 清错误不得隐式重置编码器估算器、控制器或校准流程。
5. 同时发生多个故障时必须保留 first fault 和后续因果链。
6. 所有状态命令必须有 `request_id`，并最终产生 accepted/rejected/completed/failed 结果。
7. 固件、Python、TypeScript 不得手写三份协议常量。
8. 每个跨 ISR/线程的数据对象必须使用 epoch/sequence 或明确的原子同步策略。
9. 不以 `osDelay()`、连续多次读取布尔量或增加超时作为竞态修复。
10. 重构期间保持旧 CAN 兼容层，只有 USB 功能达到等价覆盖并完成迁移后才能删除旧接口。

## 5. Phase 0：建立基线和保护网

### 目标

在改变行为前，记录当前构建、协议、硬件失败和测试覆盖情况。

### 工作项

1. 保存工作区 diff，并分类现有实验修改。
2. 建立三个 CMake 构建配置：
   - `firmware-release`：`-O2`。
   - `firmware-debug`：`-Og -g3`，保留符号，不要求用户使用 GDB。
   - `host-debug`：本机编译，启用 ASan/UBSan（支持时）。
3. 把现有 C++ 测试接入 CMake/CTest；当前 `Firmware/Tests` 中的 C++ 测试不能继续游离在构建之外。
4. 记录当前 CAN 协议版本、消息列表、扩展 item 列表和总线负载。
5. 记录当前硬件复现：进入 state 8 的请求、heartbeat、详细错误和固件 build ID。
6. 生成初始 `build_manifest.json`，至少包括 Git revision、dirty 标志、编译选项、板型和协议版本。

### 验收

- release/debug 两种固件均可构建。
- native C++ tests 可由单一命令运行。
- 基线硬件失败有原始日志，不依赖 UI 截图。
- 不改变电机运行行为。

## 6. Phase 1：统一故障模型

### 目标

先解决 `axis=CONTROLLER_FAILED, controller=0` 这种不可诊断状态。

### 核心类型

```cpp
enum class FaultSource : uint16_t;
enum class FaultCode : uint16_t;
enum class FaultSite : uint16_t;
enum class FaultSeverity : uint8_t;

struct FaultRecord {
    uint32_t fault_sequence;
    uint32_t control_sequence;
    uint32_t timestamp_cycles;
    uint32_t state_epoch;
    FaultSource source;
    FaultCode code;
    FaultSite site;
    FaultSeverity severity;
    uint32_t parent_fault_sequence;
    uint32_t arg0;
    uint32_t arg1;
    uint32_t arg2;
};
```

### 工作项

1. 实现 `FaultManager::raise(record)` 和 `FaultManager::clear(request)`。
2. first fault 一旦锁存不得被后续错误覆盖。
3. 后续错误通过 `parent_fault_sequence` 形成 consequence chain。
4. Axis/Motor/Encoder/Controller 旧错误位暂时保留，但只作为 `FaultStore` 的兼容投影。
5. 逐步替换所有直接 `error_ |= ...`；添加静态检查或 CI `rg` 门禁阻止新增直接写入。
6. 将 timeout、反馈缺失、控制器拒绝、功率级关断分别定义为不同 fault code/site。
7. `clear_errors` 拆分为：
   - `clear_faults`：只清允许清除的锁存故障。
   - `reset_estimator`：独立操作。
   - `restart_calibration`：独立操作。
8. 为 fault code/site 生成 Python、TypeScript 和 JSON schema。

### 验收测试

- 人工注入 phase velocity 缺失，根故障必须是具体 source/site，而不是泛化 Controller failed。
- 一个根故障导致 controller reject 和 disarm 时，三者形成可追踪因果链。
- 同时注入两个独立故障，两个都保留。
- 清 fault 不改变编码器 readiness epoch。
- CAN 旧错误位仍能供旧客户端读取。

## 7. Phase 2：重建 SafetySupervisor 和闭环启动事务

### 目标

消除 Axis 状态机外的隐式 `volatile bool` 状态机。

### 命令模型

```cpp
struct Command {
    uint32_t request_id;
    CommandSource source;
    CommandType type;
    CommandPayload payload;
};

struct CommandResult {
    uint32_t request_id;
    CommandStatus status; // accepted/rejected/completed/failed
    uint32_t state_epoch;
    FaultCode reason;
};
```

### 闭环进入流程

```text
SET_OPERATION(CLOSED_LOOP)
 -> validate
 -> accepted
 -> PREPARING + new state_epoch
 -> publish realtime prepare request(epoch)
 -> ISR consumes request and publishes readiness snapshot(epoch, sequence)
 -> Supervisor verifies coherent snapshot
 -> arm request(epoch)
 -> ISR/power stage confirms armed(epoch)
 -> ARMED + completed

任何失败：
 -> immediate safe output
 -> FaultRecord
 -> FAULT_LATCHED + failed(request_id, reason)
```

### 工作项

1. 将状态转换写成显式 transition table 或纯 reducer。
2. 所有 transition 包含 guard、entry action、exit action、timeout 和失败原因。
3. `ReadinessSnapshot` 使用统一 sequence/epoch，不使用多个独立 `volatile bool`。
4. ISR 与 Supervisor 之间使用单写单读 mailbox 或双缓冲快照。
5. `current_state_` 不再通过任务数组第一个元素隐式表示。
6. 校准作为 `Operation`，不能通过临时伪装 CLOSED_LOOP 复用状态。
7. timeout 生成独立 fault source/site，不直接写 Controller failed。
8. 保留兼容 AxisState 映射用于旧 CAN heartbeat。

### 验收测试

- 对每个状态和事件运行表驱动测试，覆盖所有合法和非法转换。
- 重放 encoder readiness 延迟、丢一周期、epoch 过期、控制器拒绝和 arm 失败。
- state 8 命令必须得到 completed 或带具体 reason 的 failed，不能只超时。
- 不存在 `closed_loop_*_ready_` 多布尔握手。
- `rg` 不再发现 CAN callback 直接写 `requested_state_`。

## 8. Phase 3：实时快照、事件记录与黑匣子

### 目标

让 ISR 在固定开销内提供一致观测，不直接执行通信。

### 工作项

1. 实现 `RealtimeSnapshot`，包含一次控制周期中一致的：
   - sequence/epoch；
   - encoder sample sequence；
   - phase/phase velocity；
   - position/velocity；
   - Id/Iq 和 setpoint；
   - controller output；
   - safety/operation/readiness；
   - ISR timing counters。
2. 使用双缓冲 + sequence lock，或经过证明的 SPSC 交换方案。
3. 实现固定长度 `TraceEvent`。
4. 至少分离：
   - `CriticalEventRing`；
   - `ScopeRing`；
   - `CalibrationRing`。
5. 每个 ring 有独立 overflow counter 和明确丢弃策略。
6. 实现 pre-trigger 黑匣子；first fault 后冻结关键事件并允许有限 post-trigger 采集。
7. 对 ISR 记录路径测量 worst-case cycles，并加入预算测试。

### 验收

- ISR trace 写入不调用 HAL USB、CAN、RTOS 阻塞 API、printf 或分配器。
- USB 断开运行 10 分钟不改变控制状态。
- 波形 ring 人为溢出时，critical fault 仍完整。
- Snapshot reader 不会读取跨周期字段组合。

## 9. Phase 4：USB Debug/Test 二进制协议

### 目标

把现有 stdout-only USB 改造成双向、可版本化、可恢复同步的工程协议。

### 建议帧类型

```text
HELLO / GET_SCHEMA
COMMAND / COMMAND_RESPONSE
STATE_EVENT / FAULT_REPORT
TRACE_BATCH
SCOPE_CONFIG / SCOPE_ARM / SCOPE_DATA / SCOPE_STATUS
CALIBRATION_DATA
CRASH_REPORT
PERFORMANCE_COUNTERS
```

### 工作项

1. 定义统一 frame header：magic、schema version、message type、sequence、payload length、build ID、CRC。
2. CDC 是字节流，解析器必须支持拆包、粘包、CRC 失败和重新同步。
3. USB RX 只生成 `Command`；危险操作仍由 `CommandService/SafetySupervisor` 验证。
4. USB TX task 按 critical event > state > calibration > waveform > ordinary log 排队。
5. 停用 ISR 和高优先级任务中的文本格式化。
6. 普通文本日志也应包装为协议帧，不再与裸二进制混流。
7. 复用并迁移当前 Calibration Stream 的 sequence/CRC 思路。
8. USB 未连接时只丢弃可丢数据，不允许产生控制 backpressure。

### 验收

- fuzz/属性测试覆盖随机字节、截断帧、错误长度和错误 CRC。
- 连续插拔 USB 不会影响 CAN 控制和 SafetyState。
- USB 命令有 request/response 关联，不靠 sleep 确认。
- 未知 schema/build ID 时，上位机明确拒绝错误解码。

## 10. Phase 5：波形采集和上位机

### 目标

让 UI 通过 USB 完成稳定波形和触发故障分析。

### 工作项

1. 建立生成式 `ScopeChannel` schema：ID、名称、数据类型、单位、允许采样率。
2. 采集计划在 USB task 中构建，并原子发布给 ISR；ISR 不解析字符串和任意内存地址。
3. 支持：
   - 连续低中速显示；
   - 高速 triggered capture；
   - pre-trigger/post-trigger；
   - fault/state/channel threshold 触发；
   - decimation；
   - 导出原始二进制和 CSV。
4. `foc_ui` 后端增加 `UsbDebugTransport`，与 CAN transport 解耦。
5. 前端增加：
   - Fault Inspector；
   - State Timeline；
   - Oscilloscope；
   - Test Session 页面。
6. Fault Inspector 根据 fault schema 显示 source file/line、原因、周期、样本序号和因果链。

### 验收

- 10 kHz 触发采集能够显示故障前后 phase、phase velocity、Iq 和 SafetyState。
- UI 曲线使用固件时间戳，不使用 WebSocket 到达时间对齐。
- UI 关闭或刷新不改变电机状态。
- 后端能够检测 dropped samples 和 sequence gap。

## 11. Phase 6：Crash Recorder 和无 GDB 自动定位

### 目标

HardFault/UsageFault 后重启，通过 USB 自动报告源码位置。

### 工作项

1. fault handler 首先执行硬件安全关断。
2. 不发送 USB、不格式化，保存：
   - stacked R0-R3、R12、LR、PC、xPSR；
   - MSP、PSP；
   - CFSR、HFSR、MMFAR、BFAR、ICSR；
   - active IRQ；
   - build ID；
   - SafetyState/Operation/state epoch/control sequence；
   - 最近 critical events；
   - CRC。
3. 优先保存到保留 RAM/backup SRAM，复位后上报；不要在未知故障上下文直接写 flash。
4. CMake post-build 保存匹配的 ELF、map、manifest 和 schema。
5. 后端按 build ID 查找 ELF，并自动调用 `arm-none-eabi-addr2line` 符号化 PC/LR；用户不需要 GDB。
6. 未找到完全匹配 ELF 时显示地址但明确标记“不可可靠符号化”。

### 验收

- 人工触发 HardFault，设备安全复位后 UI 显示 fault registers、PC 和正确源码位置。
- 使用错误 build 的 ELF 时不会静默显示错误行号。
- USB 不连接时 crash record 在下一次可连接启动中仍可读取（在所选保留介质能力范围内）。

## 12. Phase 7：精简 CAN 产品协议

### 目标

在 USB 已覆盖诊断能力后，收缩 CAN。

### 推荐保留

1. Heartbeat/Status：设备状态、operation、紧凑 fault summary、position/velocity/torque。
2. 高频 setpoint：torque/velocity/position/MIT。
3. 管理命令：enable/disable/clear fault/start operation。
4. 管理 ACK：request ID、结果、state epoch、简化 reason。
5. 少量稳定配置；复杂配置优先 USB，或使用独立低频配置事务。

### 工作项

1. 新建单一协议 schema，并生成 C++/Python/TypeScript。
2. 禁止 CAN handler 访问调试内部变量。
3. 后端缓存按帧保留 timestamp/sequence，不再合成伪原子状态。
4. `clear_fault` 在固件 ACK 前不清本地缓存。
5. 迁移脚本和 UI 后，删除扩展 CAN 中的 waveform、Vernier 内部 item、debug counter 和重复配置 setter。
6. 更新协议版本和兼容矩阵。
7. 测量 1 Mbps 下典型和最坏总线利用率。

### 验收

- 标准运动控制完全不依赖 USB。
- USB 调试完全不依赖扩展 CAN item。
- 断开 USB 后 CAN 运行行为不变。
- 所有状态命令都有确定结果。
- 固件、后端、前端协议版本一致。

## 13. Phase 8：Python 测试架构

### 目标

测试需求不再推动产品 CAN 膨胀。

### 分层

```text
L0 Native Unit:
  PID/PLL/Vernier/fault/protocol codec

L1 Native Scenario:
  virtual clock + sensor replay + supervisor transitions

L2 Target Loopback:
  MCU、DMA、USB parser/ring，无功率输出

L3 HIL:
  实际编码器、电机、校准、闭环和保护

L4 Product CAN Compatibility:
  只验证公开 CAN 行为和负载
```

### 工作项

1. 提取不依赖 HAL/RTOS/全局 `odrv` 的：
   - `control_core`；
   - `encoder_core`；
   - `supervisor_core`；
   - `fault_core`；
   - `protocol_core`。
2. 通过构造参数/接口注入 Clock、SensorSource、PowerStage、NVM、TraceSink。
3. 提供 C ABI 或 pybind11，使 Python 能推进虚拟时间、注入样本和读取结果。
4. 实现共享 Python package，替代每个脚本复制 CAN ID、错误表和 WebSocket client。
5. HIL 使用 USB Test Session：begin、capabilities、command、subscribe、stop、end。
6. 所有危险 HIL 测试有速度/电流上限、超时、finally safe stop 和独立硬件急停要求。
7. 支持保存真实 snapshot/trace，并在 native test 中 replay。
8. 逐个迁移现有 32 个 Python 文件；迁移完成前保留兼容 wrapper。

### 验收

- 状态机、反馈延迟和 timeout 场景可在无硬件 PC 上确定性复现。
- Python 能注入“指定 sequence 丢失/CRC 错误/反馈过期”。
- HIL 脚本不需要读取私有 CAN debug item。
- 协议 golden vectors 同时验证 C++、Python、TypeScript。
- CI 能运行 L0/L1，硬件机运行 L2/L3。

## 14. Phase 9：删除旧架构和最终硬化

### 删除条件

只有同时满足以下条件，才能删除旧接口：

1. USB 功能达到对等覆盖。
2. 对应 Python/UI 已迁移。
3. 有至少一次完整硬件回归记录。
4. CAN 兼容策略已确定。

### 清理项

- 旧的多个 readiness `volatile bool`。
- CAN 中的内部诊断 item 和重复 setter。
- stdout CSV waveform。
- 重复错误枚举和手写协议版本。
- 直接写 `error_`、`requested_state_` 和本地状态缓存的路径。
- 依赖 sleep 判断命令完成的测试。
- 失去构建入口的旧测试。

### 最终回归

1. 冷启动、重复 enable/disable 1000 次。
2. USB 连续插拔和主机进程崩溃。
3. CAN 丢包、延迟、bus-off 和主机失联。
4. 编码器 CRC/固定位/样本过期/双编码器分歧。
5. 过流、过压、欠压、温度和功率级故障。
6. 闭环进入每个失败点的 fault site 验证。
7. 波形满载下 ISR deadline 和 CAN 行为。
8. HardFault/UsageFault/看门狗复位后的 crash report。

## 15. 每阶段必须提交的证据

执行者每完成一个 Phase，应提供：

1. 变更文件列表和设计说明。
2. `git diff --check`。
3. 构建命令和完整结果摘要。
4. native test/CTest/pytest 结果。
5. 新增测试对应的需求或故障。
6. 若改变 ISR：worst-case cycle 测量。
7. 若改变协议：schema diff、golden vectors 和兼容说明。
8. 若改变硬件行为：固件 build ID、接线/电源条件、原始运行日志和安全停止结果。
9. 尚未验证的风险，不得用“应该可以”标记完成。

## 16. 审查时的否决项

出现以下任何一项，应停止合并并返工：

- 新增用于握手的裸 `volatile bool`。
- 在 ISR 中调用 USB、CAN、printf、malloc 或阻塞 RTOS API。
- 为了测试新增私有 CAN debug item。
- 用延时或重复读取掩盖 race。
- 直接写父级错误位而不产生 FaultRecord。
- 一个枚举同时表示安全状态、操作、运动趋势和故障。
- USB 断开导致控制状态变化。
- 波形缓存能够覆盖 first-fault 记录。
- 命令发送后后端未经 ACK 就更新状态缓存。
- 固件、Python、TypeScript 各自维护协议常量。
- HardFault 使用不匹配 build ID 的 ELF 进行静默符号化。

## 17. 完成定义

整体重构只有在以下条件全部满足时才算完成：

1. 当前 state 8 启动失败能够通过 USB 报告唯一根故障、源码位置、控制周期和现场波形。
2. CAN 协议不再包含 Vernier 内部细节、波形或临时 debug counter。
3. CAN 单独连接时可安全完成产品运行控制。
4. USB 单独连接时可完成诊断、波形、校准和测试，但不能绕过 SafetySupervisor。
5. 状态命令都有 request ID 和最终结果。
6. 清 fault 不重置无关子系统。
7. Python 可以在 PC 上确定性测试状态机和主要控制核心。
8. HardFault 重启后 UI 能自动定位到匹配构建的函数/源码位置。
9. 不再存在 `axis error != 0` 但没有来源和 fault site 的情况。
10. 所有协议和诊断 schema 由单一事实源生成。

## 18. 推荐执行方式

不要在一个超大提交中完成全部重构。建议每次只执行一个 Phase，并在通过该 Phase 的验收门槛后再继续。
Phase 0、1、2 是解决当前闭环失败不可诊断问题的最小闭环；Phase 3、4、5 建立 USB 调试能力；
Phase 6、7、8、9 完成无 GDB 定位、CAN 收缩和测试迁移。
