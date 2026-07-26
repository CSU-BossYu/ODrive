# ODrive 单轴双编码器多速率控制改造计划

> 适用项目：ODrive v3.6 / STM32F405 / DRV8301 / 单轴 / 双 MT6826S Vernier / 42:1 输出轴控制  
> 文档目的：作为固件设计、实施、评审和实机验收的交接依据  
> 推荐目标：24 kHz PWM、12 kHz 电流环、6 kHz Vernier、2 kHz 速度环、1 kHz 位置环

> 修订状态：实施规范 v2。v2 增加并发所有权、样本时间戳、控制命令一致性、SPI callback 边界和控制模式延迟契约；第一轮改造明确保持现有 Vernier 数学语义。

> 实施状态（2026-07-15）：阶段0A的软件计时基础已实现并通过固件编译；尚未刷板采集8 kHz实测基线，因此阶段0A退出条件尚未满足，0B控制语义改造不得据此视为已放行。

阶段0A新增CAN诊断（扩展子命令`0x0A`）：`0x2E/0x2F`为pair请求到发布的当前/最大周期数，`0x31/0x32/0x33`为完整控制回调当前/最大周期数及样本数，`0x39..0x3B`为50%/70%/85%预算越界次数，`0x3C`为周期计数器频率，`0x3D/0x3E`为主编码器发布到消费的当前/最大样本年龄。配套读取脚本为`Firmware/Tests/hex_4342_mt6826s/read_mt6826s_pair.py`。

> 8 kHz空闲态初测（2026-07-15）：控制回调约25.77 us、最大25.83 us；pair稳定约94.2～96.6 us、启动以来最大211.36 us；主样本年龄稳定约69～72 us、启动以来最大152.70 us；约26.8万次pair完成期间busy累计4次，CRC、DMA、ADC、FOC timing及deadline错误均为0。该数据来自`AXIS_STATE_IDLE`，且当前计时范围尚未覆盖`ControlLoop_IRQHandler`内控制回调前后的ADC/DC校准/PWM更新段，因此只能作为空闲态初步基线，不能作为12 kHz放行依据。

> 校准完成后的第二次空闲态窗口：CSV实际包含18点；pair为94.21～96.70 us（平均94.91 us，p95/p99均96.52 us），控制回调为26.66～26.67 us，主样本年龄为69.46～72.35 us。窗口内pair成功增加13666次、busy增量为0，所有ADC/FOC/deadline错误增量为0。启动累计busy为9915、pair最大211.36 us、样本年龄最大176.45 us，这些累计值发生在本窗口之前，不能归因于稳定空闲运行。

> 正式8 kHz静止态基线（100点、9.940 s）：有效pair频率7999.3 Hz；pair平均94.737 us、p95 96.500 us、p99 96.536 us、窗口最大96.655 us；控制回调平均27.465 us、p99/最大27.512 us；主样本年龄平均71.307 us、p95/p99/最大72.387 us。窗口内pair完成增加79527次，busy、CRC、DMA、ADC、FOC timing、deadline及50/70/85%预算越界的增量全部为0。启动累计pair最大219.202 us、年龄最大176.452 us、控制回调最大28.542 us在整个窗口内均未变化，属于窗口之前的历史事件。编码器角度和速度全程不变；CSV未记录axis state，因此此结果只标记为静止态，不能区分IDLE与闭环保持。

## 1. 改造结论

建议采用多速率控制，但不要把双 MT6826S 完整 pair 读取作为电流环每周期必须完成的同步前置条件。

第一目标配置：

| 模块 | 目标频率 | 周期 | 说明 |
| --- | ---: | ---: | --- |
| PWM | 24 kHz | 41.667 us | 保持现有 PWM 频率 |
| ADC 电流采样与 FOC | 12 kHz | 83.333 us | 由 `TIM_1_8_RCR=1` 得到 |
| 主编码器有效更新 | 12 kHz 目标 | 83.333 us | pair 与 main-only 交替采样 |
| 主辅 Vernier pair | 6 kHz | 166.667 us | 每两个快速周期一次 |
| 输出轴速度环 | 2 kHz | 500 us | 每六个快速周期一次 |
| 输出轴位置环/轨迹 | 1 kHz | 1 ms | 每十二个快速周期一次 |
| ADRC/DOB/摩擦补偿 | 2 kHz | 500 us | 与速度/转矩伺服同步 |
| 温度与慢速维护 | 100 Hz～1 kHz | 1～10 ms | 从快速 ISR 移出 |

保守回退配置：24 kHz PWM、8 kHz 电流环、4 kHz Vernier、2 kHz 速度环、1 kHz 位置环。

暂不建议直接进入 24 kHz 电流环。必须先完成多速率解耦、SPI 实测和最坏执行时间测量，再单独评估。

## 2. 改造目标与非目标

### 2.1 目标

1. 降低电流采样到 PWM 更新的延迟，并为以后提高电流带宽保留空间。
2. 让位置、速度、轨迹、ADRC 和摩擦补偿按机械系统需要的频率运行。
3. 主编码器持续为 FOC 提供新鲜或可预测的电角度。
4. 辅助编码器只承担 Vernier 绝对分支、输出轴校正和一致性诊断，不阻塞 FOC。
5. 保持当前 CAN 协议、控制模式和用户参数语义不变。
6. 所有分频、丢样、超时和过载均可诊断并可安全停机。
7. 改造可分阶段验证，任一阶段均可回退到已验证配置。

### 2.2 非目标

本次不同时完成以下工作：

- 不改变功率板、DRV8301、ADC采样电路和电流传感器硬件。
- 不直接提升 PWM 到 32/48 kHz。
- 不在同一变更中重新整定所有控制增益。
- 不同时重写 Vernier 数学算法。
- 不同时引入 MPC、全阶机械观测器或新的通信协议。
- 不把本次频率改造与转矩常数、摩擦参数最终标定绑定在同一提交中。

## 3. 当前架构与必须解决的问题

### 3.1 当前时基

现有定时器参数：

```text
TIM clock       = 168 MHz
period clocks   = 3500
PWM frequency   = 168 MHz / (2 * 3500) = 24 kHz
RCR             = 2
control rate    = 24 kHz / (RCR + 1) = 8 kHz
```

`CURRENT_MEAS_PERIOD` 同时被编码器 PLL、轨迹、位置积分、速度积分、ADRC、摩擦斜率和超速计时使用。改成多速率后不得继续用一个全局 `current_meas_period` 表示所有模块周期。

### 3.2 当前闭环流水线

现有 `control_loop_cb()` 每个快速周期执行：

```text
清空全部输出端口
  -> 安全检查
  -> Encoder::update()
  -> Controller::update()
  -> Motor::update()
  -> CurrentControl::update()
```

当前每周期清空 `torque_output_`、`phase_`、`pos_estimate_`、`Idq_setpoint_` 等端口。若只给 `Controller::update()` 增加分频条件，未运行控制器的周期将没有有效转矩输出，最终导致闭环失败或电机被解除使能。

因此本改造的首要要求是：

> 为跨速率信号建立显式零阶保持和新鲜度检查，不能依赖原有“本周期必须重新发布”的端口语义。

### 3.3 当前双 SPI 路径

MT6826S 每次 burst 为 6 字节。生产配置 SPI3 分频为 8；按 42 MHz APB1 SPI 时钟估算：

```text
SPI clock       = 42 MHz / 8 = 5.25 MHz
single transfer = 48 / 5.25 MHz = 9.14 us
main + aux raw  = 18.29 us
```

加入 CS、DMA、回调、CRC 和 arbiter 切换后，完整 pair 预算先按 20～30 us 设计，最终必须用逻辑分析仪和固件时间戳实测。

在 12 kHz 快速周期 83.33 us 下，完整 pair 有可接受裕量；在 24 kHz 周期 41.67 us 下，虽然原始传输能放入，但系统最坏执行时间和中断抖动裕量不足，不应未经测量直接上线。

### 3.4 并发上下文与所有权

| 数据 | 唯一写入者 | 读取者 | 发布方式 | ISR内允许工作 |
| --- | --- | --- | --- | --- |
| ADC电流快照 | ADC/电流ISR | fast FOC | 现有同步时序 | 有界转换与发布 |
| main raw样本 | SPI DMA callback | 编码器fast阶段、诊断 | 双缓冲mailbox | 复制raw、时间戳、计数 |
| pair raw样本 | pair DMA callback | Vernier消费阶段、诊断 | 整体双缓冲mailbox | 复制main+aux整体、时间戳 |
| 控制命令 | CAN/协议命令处理 | 2 kHz/1 kHz外环 | ControlInputMailbox | 协议侧仅组包和发布 |
| 输出轴状态 | 编码器控制阶段 | 外环、CAN | 一致快照 | 不在DMA callback计算 |
| held torque | 2 kHz torque阶段 | 12 kHz Motor阶段 | 单生产者快照 | fast只读与年龄检查 |
| 安全覆盖状态 | fast安全路径/timeout | 所有控制阶段 | 高优先级latch | 可直接置零/解除使能 |

基本约束：单份状态只有一个逻辑写入者；跨上下文只传不可变快照；CAN/USB只读诊断快照，不直接读取正在被ISR修改的复合对象。

## 4. 目标控制架构

```text
                          24 kHz PWM
                               |
                    12 kHz ADC + fast tick
                               |
          +--------------------+--------------------+
          |                    |                    |
   main encoder/PLL      current control       fast safety
       12 kHz target         12 kHz              12 kHz
          |                    ^
          +---- phase ---------+
          |
     pair every 2 ticks
          |
   Vernier resolver 6 kHz
          |
   output pos/vel snapshot
          |
     velocity/torque 2 kHz
          |
     position/trajectory 1 kHz
```

### 4.1 信号保持规则

以下信号使用零阶保持，直到生产者发布新序号：

| 信号 | 生产者 | 消费者 | 最大允许年龄建议 |
| --- | --- | --- | ---: |
| `motor_phase` / `phase_vel` | 主编码器快速估计器 | FOC | 2 个 fast tick；期间必须预测 |
| `output_pos` / `output_vel` | Vernier/输出估计器 | 外环 | 2 个 pair 周期 |
| `velocity_torque_cmd` | 2 kHz 速度/转矩控制 | 12 kHz Motor/FOC | 1 个 velocity 周期 + 1 fast tick |
| `position_vel_cmd` | 1 kHz 位置环 | 2 kHz 速度环 | 1 个 position 周期 + 1 velocity tick |
| `Idq_setpoint` | Motor::update | 电流控制器 | 1 个 fast tick |

每个跨速率快照至少包含：

```cpp
struct TimedSample {
    Value value;
    uint32_t sequence;
    uint32_t request_ticks;
    uint32_t completion_ticks;
    uint32_t publish_ticks;
    bool valid;
};
```

这里的结构只描述负载，不代表并发安全。发布必须采用“双缓冲 + 单一原子发布索引”，第一轮不采用多个字段独立写入：

```cpp
template<typename T>
struct IsrMailbox {
    T slots[2];
    volatile uint32_t published_index;
};
```

写入者只写非活动槽，完整写完后在极短临界区内更新 `published_index`。读取者先锁存索引，再复制完整对象；浮点解算、CRC解释、PLL和Vernier计算不得位于临界区。若编译器/平台不能证明发布屏障，应使用项目CPU临界区原语，而不能只依赖 `volatile`。

pair 必须作为一个整体发布：

```cpp
struct RawPairSample {
    RawSample main;
    RawSample aux;
    uint32_t pair_sequence;
    uint32_t request_ticks;
    uint32_t main_completion_ticks;
    uint32_t aux_completion_ticks;
    uint32_t publish_ticks;
    bool valid;
};
```

禁止分别发布 main/aux 后再由消费者拼接，否则可能混合不同 pair。

消费者应区分：

- 新样本：更新状态和序号。
- 合法旧样本：执行预测或零阶保持。
- 超龄样本：进入降级状态或解除使能。
- 无效样本：按现有错误策略处理。

### 4.2 快速电流环

12 kHz 快速循环只保留：

1. ADC电流读取和校准。
2. 主编码器快照锁存及电角度预测。
3. `Motor::update()`：将保持的转矩命令换算为 `Iq`，更新电压前馈。
4. FOC电流PI与SVPWM。
5. 电流、母线、栅极、时序和紧急关断检查。
6. 必要的计数器和最坏执行时间测量。

不得在每个快速周期执行：

- 梯形轨迹计算。
- 位置积分器。
- 速度积分器。
- ADRC三阶状态更新。
- 摩擦模型指数运算。
- 温度慢速滤波。
- CAN/USB格式化和日志输出。

### 4.3 编码器采样计划

推荐 12 kHz 快速 tick 下交替执行：

```text
偶数 fast tick: main + aux 完整 pair
奇数 fast tick: main-only
```

结果：

```text
main sample effective target = 12 kHz
aux sample effective target  = 6 kHz
Vernier pair target          = 6 kHz
```

实现约束：

1. pair 内主辅必须保持背靠背读取，以限制两编码器时间偏差。
2. main-only 读取不得与尚未结束的 pair 抢占 SPI arbiter。
3. 若 tick 到来时 SPI busy，本周期跳过启动，使用估计值继续，不因一次 busy 立即报错。
4. 连续 busy、样本超龄或 CRC 错误达到阈值后才降级或停机。
5. 每次采样记录“请求时间、主完成时间、辅完成时间、pair完成时间”。
6. DMA完成callback只发布有界的raw快照和时间戳，不执行浮点Vernier、PLL、重锁或输出轴更新。
7. 控制环编码器阶段只在新 pair sequence 到达时运行Vernier锁定、分支判断和残差诊断；不得重复把旧 pair 当成新观测。
8. 主编码器 PLL/unwrap 在新 main sequence 到达时校正，其他快速周期按真实elapsed time预测。
9. mailbox满时采用“最新样本覆盖旧未消费样本”，增加覆盖计数；持续覆盖再升级为degraded/fault。
10. 有效更新率按“成功发布的新sequence数/真实时间”计算，不按请求数计算。

### 4.3.1 第一轮必须保持的 Vernier 语义

本轮选择路线A，不引入pair校正输出观测器：

- 主编码器PLL在fast tick连续传播。
- pair只负责初始绝对分支锁定、unwrap一致性、残差诊断和重锁判断。
- 输出轴位置与速度仍从主编码器连续估计按主减速比换算。
- 新pair不得对连续输出位置施加未定义的校正增益。
- pair恢复不得造成 `1/42` 输出圈跳变。

如果未来需要“pair到达时校正输出状态”，必须作为独立算法项目定义校正增益、连续性、稳定性和回归测试，不属于本轮频率改造。

如果 main-only 与 pair 状态机改造风险过高，阶段一允许所有编码器仍以完整 pair 6 kHz运行，FOC在中间周期使用相位预测；但必须验证最高电机速度下的 `Id`、相位残差和电流稳定性。

### 4.4 外环拆分

建议将当前单体 `Controller::update()` 拆成三个明确阶段：

```cpp
bool update_setpoints(float dt, bool position_tick);
bool update_position(float dt_position);
bool update_velocity_torque(float dt_velocity);
```

职责建议：

#### 1 kHz位置/轨迹阶段

- `TRAP_TRAJ`时间推进和位置/速度前馈生成。
- `POS_FILTER`二阶输入滤波。
- 位置误差、位置死区、位置积分。
- 位置P输出到缓存的速度修正。
- 位置相关增益调度状态。

#### 2 kHz速度/转矩阶段

- `VEL_RAMP`、`TORQUE_RAMP`和MIT输入处理。
- 使用缓存的位置速度修正形成 `vel_des`。
- 速度PI和速度限制。
- ADRC/DOB状态更新。
- 摩擦补偿与斜率限制。
- 转矩限制、积分抗饱和和功率估计。
- 发布带序号的保持转矩命令。

#### 12 kHz电机阶段

- 读取最近一次有效转矩命令。
- 按减速比和转矩常数换算电机 `Iq`。
- `R/L`、反电动势前馈。
- 电流矢量限幅。

对 PASSTHROUGH 模式可以在2 kHz阶段直接更新；CAN命令接收仍异步，只写入线程安全的输入邮箱。

### 4.5 控制命令一致性与优先级

多速率前必须引入统一 `ControlInputMailbox`。CAN、MIT、step/dir及其他命令源不得逐字段直接修改外环正在消费的状态。

```cpp
struct ControlCommandSnapshot {
    ControlMode control_mode;
    InputMode input_mode;
    float pos;
    float vel;
    float torque;
    float kp;
    float kd;
    uint32_t received_ticks;
    uint32_t sequence;
    bool valid;
};
```

规则：

1. 一个协议帧对应一次完整快照发布。
2. 外环只在固定调度边界消费完整快照。
3. timeout、quick-stop和紧急torque-zero不是普通邮箱命令，必须通过更高优先级安全覆盖路径生效。
4. 安全覆盖生效后，旧普通命令不得再次覆盖；必须等待新sequence或重新使能条件。
5. 模式切换与该模式第一条setpoint必须定义原子边界，不能出现新模式配旧setpoint。

### 4.6 控制模式调度与延迟契约

第一轮目标契约如下，若实测或产品要求不同，实施前必须显式修改表格：

| 模式/路径 | 输入消费 | 状态推进 | 转矩生成 | 最大正常command-to-torque延迟 | 安全覆盖 |
| --- | ---: | ---: | ---: | ---: | ---: |
| TORQUE+PASSTHROUGH | 2 kHz | 2 kHz | 2 kHz | 0.5 ms + 1 fast tick | fast tick |
| VELOCITY/PASSTHROUGH | 2 kHz | 2 kHz | 2 kHz | 0.5 ms + 1 fast tick | fast tick |
| VEL_RAMP | 2 kHz | 2 kHz | 2 kHz | 0.5 ms + 1 fast tick | fast tick |
| TORQUE_RAMP | 2 kHz | 2 kHz | 2 kHz | 0.5 ms + 1 fast tick | fast tick |
| POS_FILTER | 1 kHz输入状态 | 1 kHz位置、2 kHz速度 | 2 kHz | 1 ms + 1 velocity tick | fast tick |
| TRAP_TRAJ | 1 kHz | 1 kHz位置、2 kHz速度 | 2 kHz | 1 ms + 1 velocity tick | fast tick |
| MIT | 2 kHz第一轮 | 2 kHz | 2 kHz | 0.5 ms + 1 fast tick | fast tick |

若MIT或纯转矩接口需要更低延迟，应在8 kHz多速率验证后单独增加fast command path，而不是让全部Controller逻辑回到fast频率。

## 5. 时间基准改造

### 5.1 新增明确时基

禁止控制器继续隐式使用单一 `current_meas_period`。建议集中定义：

```cpp
constexpr float pwm_dt      = 1.0f / 24000.0f;
constexpr float fast_dt     = 1.0f / 12000.0f;
constexpr float pair_dt     = 1.0f / 6000.0f;
constexpr float velocity_dt = 1.0f / 2000.0f;
constexpr float position_dt = 1.0f / 1000.0f;
```

最好由定时器参数和整数分频在编译期推导，而不是重复写死数值，并增加 `static_assert`：

```text
PWM_HZ % FAST_HZ == 0
FAST_HZ % PAIR_HZ == 0
FAST_HZ % VELOCITY_HZ == 0
FAST_HZ % POSITION_HZ == 0
VELOCITY_HZ % POSITION_HZ == 0
```

### 5.2 必须逐项替换的 `dt`

以下逻辑必须使用所属环路真实周期：

- 电流PI积分：`fast_dt`。
- 电机模型和相位预测：实际时间戳差或 `fast_dt`。
- 编码器PLL预测：`fast_dt`；观测校正只在新主样本时执行。
- 输出轴Vernier速度更新：实际 pair 时间差。
- 速度积分器：`velocity_dt`。
- 位置积分器：`position_dt`。
- `VEL_RAMP`/`TORQUE_RAMP`：实际执行该逻辑的周期。
- `POS_FILTER`、轨迹和调谐信号：`position_dt`或所选固定周期。
- ADRC/DOB：`velocity_dt`，并按2 kHz离散化重新审查稳定性。
- 摩擦转矩 slew rate：`velocity_dt`。
- 超速持续时间：使用真实速度环时间或时间戳，不再按 fast tick假定。
- 功率低通：使用其实际执行周期。
- watchdog：保持真实秒单位，避免随循环频率改变超时时间。

### 5.3 滤波器和观测器

任何按 `dt`生成系数的滤波器必须在频率改变后重新计算。禁止沿用8 kHz下的离散系数。

降采样前应进行抗混叠处理。速度环2 kHz时，建议输出速度估计有效低通带宽不高于200～300 Hz；实际控制带宽先保持25～40 Hz。

## 6. 定时器与ISR改造

### 6.1 阶段一目标

将：

```cpp
#define TIM_1_8_RCR 2
```

改为：

```cpp
#define TIM_1_8_RCR 1
```

由此得到12 kHz快速周期。实施者必须审计所有包含 `(TIM_1_8_RCR + 1)` 的：

- ADC采样时间戳。
- `TIM1_INIT_COUNT`相关补偿。
- PWM更新时间戳。
- `MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA`。
- TIM1/TIM8同步和`timestamp_`截止检查。
- `CURRENT_MEAS_PERIOD`与编译期频率。

不能只修改宏后直接认为时序正确。

### 6.2 ISR优先级原则

1. ADC/电流/FOC截止时间最高。
2. SPI DMA完成中断不得长时间抢占电流控制关键区。
3. sampling ISR只发起异步事务，不做Vernier浮点解算。
4. pair解析、CRC后的状态发布应保持有界执行时间。
5. CAN、USB、日志不得在电流截止路径中执行。

### 6.3 最坏执行时间

现有 `TaskTimer` 只保留有限测量信息。建议新增：

- 每个模块当前时长。
- 启动以来最大时长。
- 超过50%、70%、85%周期预算的计数。
- fast ISR总体最大时长。
- SPI main、aux、pair的最大事务时长。
- 外环最大时长。

12 kHz验收建议：

```text
fast ISR WCET <= 50 us 目标
fast ISR WCET <= 58 us 强制上限（约70%周期）
deadline miss = 0
```

剩余裕量用于异步中断、温漂、编译差异和未来功能。

## 7. 诊断与CAN可观测性

新增或扩展以下只读诊断项：

```text
pwm_rate_hz
fast_loop_rate_hz
pair_target_rate_hz
pair_actual_rate_hz
velocity_loop_rate_hz
position_loop_rate_hz
fast_loop_max_ticks
velocity_loop_max_ticks
position_loop_max_ticks
spi_main_max_ticks
spi_pair_max_ticks
main_sample_age_ticks
pair_sample_age_ticks
main_sample_skip_count
pair_sample_skip_count
enc_main_ok_count
enc_pair_ok_count
enc_crc_error_count
cl_deadline_miss_count
held_torque_age_ticks
```

协议扩展必须向后兼容，优先放入现有扩展诊断子命令，不改变已冻结常用CAN帧布局。

## 8. 安全与故障策略

### 8.0 统一故障状态机

编码器数据通路统一使用：

```text
NORMAL -> PENDING -> DEGRADED/SUSPECT -> FAULT
                    ^                 |
                    +---- RELOCK <----+
```

语义：

- `NORMAL`：样本年龄和残差均满足要求。
- `PENDING`：一次预期busy、事务在飞行或短暂未消费，不降额。
- `DEGRADED`：主样本暂缺但仍可预测，或pair暂缺但主传播有效；允许限时、可选择限矩。
- `SUSPECT`：pair残差位于接受和拒绝阈值之间，保持连续输出但不宣称完整锁定。
- `FAULT`：主样本超龄、持续CRC/DMA失败、deadline或不可恢复分支错误；立即解除使能。
- `RELOCK`：仅对辅助/pair恢复使用，满足连续确认帧后回到NORMAL；不得让主编码器硬故障自动恢复带电状态。

每个状态的时间阈值必须用真实时间或样本时间戳表示；不得因8/12 kHz切换而改变物理超时。

### 8.1 主编码器

- 单次无新样本：按速度预测，状态保持正常。
- 连续2个 fast tick无新样本：标记degraded，限制允许电角速度或转矩。
- 超过配置的最大年龄：立即解除使能并报告主编码器超时。
- CRC、固定bit或SPI错误：不使用该次样本校正PLL。

最大年龄最终应结合最高机械转速和14极对电机计算，不应只用固定帧数拍脑袋决定。

### 8.2 辅助编码器/pair

- 单次pair缺失：继续使用主编码器传播输出位置。
- 连续pair缺失：进入Vernier suspect/degraded。
- 超过现有容忍帧数或残差持续超限：解除使能。
- pair恢复后必须经过现有重锁确认，不得单帧直接恢复完整可信状态。

### 8.3 外环命令

- 速度/转矩输出超龄：先执行现有quick stop/torque zero策略。
- 位置输出超龄：冻结位置速度修正，不允许无限保持旧轨迹推进。
- 控制loop分频计数异常或序号倒退：解除使能。

### 8.4 转矩常数注意事项

当前 `torque_constant=0.04 Nm/A` 尚未可靠标定。频率改造阶段不得同时提高电流限制或激进提高外环增益。所有实机测试先使用保守电流/转矩上限；转矩常数和输出侧有效转矩常数应另行标定。

## 9. 分阶段实施计划

### 阶段0A：可信测量基础设施

保持8 kHz、现有算法和现有SPI调度不变，先让后续验收指标可测。

1. 修正/扩展`TaskTimer`：启用max统计，增加fast ISR全程计时和超预算桶。
2. 明确计时器频率、回绕处理、读取原子性和测量自身开销。
3. 增加SPI生命周期时间戳：request、main CS/DMA完成、aux完成、pair publish、consumer consume。
4. 增加main/pair request、complete、publish、consume、overwrite、busy、CRC和DMA失败计数。
5. 增加样本age当前值、max和分桶；主机端计算p50/p95/p99。
6. 诊断只观测，不改变现有控制语义。

退出条件：能够可信地证明8 kHz基线WCET、SPI实际耗时、样本年龄和deadline状态。

### 阶段0B：并发数据契约

仍保持8 kHz和现有控制频率。

1. 实现main raw与完整pair raw双缓冲mailbox。
2. DMA callback缩减为有界raw发布，不做PLL、Vernier或输出轴状态计算。
3. 在控制环编码器阶段消费新sequence并执行现有数学算法。
4. 实现`ControlInputMailbox`，一个命令帧原子发布一份完整快照。
5. 实现安全覆盖latch，确保timeout/quick-stop/torque-zero不能被旧命令覆盖。
6. 为mailbox覆盖、撕裂读取、sequence回绕和并发命令增加测试。

退出条件：DMA/协议写入者与控制消费者之间不存在复合对象撕裂；行为与原8 kHz基线等价。

### 阶段0C：冻结基线与采集现状

交付物：基线固件、参数备份、日志和测试报告。

1. 保存当前可稳定运行的固件和配置。
2. 记录8 kHz下所有 `TaskTimer`、deadline和SPI计数。
3. 完成以下基准测试：
   - 空载静止10分钟。
   - 正反向低速恒速。
   - 速度阶跃。
   - 位置阶跃。
   - MIT小信号。
   - 最大计划转速运行。
4. 保存 `Iq_setpoint/Iq_measured/Id_measured`、位置、速度、主辅角度、pair序号和错误计数。
5. 明确验收用电机、母线电压、负载和温度条件。

退出条件：现有8 kHz基线可重复，deadline miss为0，日志可用于A/B比较。

### 阶段1：只做外环多速率，电流环保持8 kHz

目标配置：8 kHz电流、4 kHz pair、2 kHz速度、1 kHz位置。

1. 引入fast/velocity/position tick和编译期分频。
2. 引入转矩命令零阶保持及序号/年龄。
3. 按第4.6节模式矩阵拆分`Controller::update()`。
4. 保留fast安全覆盖；明确TORQUE PASSTHROUGH和MIT的最大延迟。
5. 所有积分器、ramp、ADRC、摩擦、watchdog和超速计时改用正确`dt`或真实时间。
6. SPI error-rate区分未请求、in-flight、expected busy、DMA失败、CRC失败、未消费和超龄。
7. 保持电流环带宽和所有控制增益不变。
8. 完成单元测试：8 kHz旧架构与多速率架构在无噪声模型下的低频输出应接近。

退出条件：所有原控制模式可用；未运行外环的fast tick仍持续发布有效`Iq`；无端口新鲜度误判。

### 阶段2：拆分主编码器与完整pair调度

1. 给编码器采样器增加显式采样计划：`MAIN_ONLY`、`MAIN_AUX_PAIR`、`NONE`。
2. 完善SPI arbiter busy行为和计数。
3. 主样本与pair样本分别记录序号、时间戳和有效性。
4. PLL只在新main样本到达时校正，其他tick只预测。
5. Vernier resolver只在新pair时更新。
6. 严格保持第4.3.1节路线A语义：主PLL连续传播，pair只锁定/诊断/重锁，不引入新校正器。
7. 在arbiter层保证pair原子性：pair进行期间保留总线，aux续接优先，main-only不得插入。
8. expected contention不设置sticky SPI fault；真实DMA/协议错误单独报告。
9. 用逻辑分析仪测量request到CS、main到aux间隔、pair publish到consume及FOC实际使用延迟。

退出条件：main有效更新率达到目标的99.9%以上；pair更新率达到目标的99.5%以上；无持续busy；CRC错误率满足生产要求。

### 阶段3：电流环从8 kHz提高到12 kHz

1. 设置`TIM_1_8_RCR=1`。
2. 重新验证ADC、TIM1/TIM8和PWM更新时间戳。
3. 电流带宽仍保持1500 rad/s（约239 Hz）。
4. 主编码器12 kHz目标，pair 6 kHz，速度2 kHz，位置1 kHz。
5. 比较8 kHz与12 kHz电流阶跃和速度阶跃。
6. 检查最高转速下的`Id_measured`、`Iq`纹波和相位残差。

退出条件：deadline miss为0；fast ISR满足WCET；MOSFET和电机温升无异常；高低速均无新振荡。

### 阶段4：选择性提高电流带宽

仅当12 kHz执行频率已经验证，且实测显示239 Hz电流带宽限制性能时实施。

推荐顺序：

```text
1500 rad/s (239 Hz)
-> 1900 rad/s (302 Hz)
-> 2500 rad/s (398 Hz)
```

每级测试：

- 正负`Iq`阶跃。
- 10%～90%上升时间。
- 超调和稳定时间。
- `Id`耦合。
- 电压饱和与积分恢复。
- 母线不同电压。
- 冷机/热机。

发现噪声、超调、啸叫或相位裕度不足即退回上一级。

### 阶段5：是否进入24 kHz电流环的决策门

只有同时满足以下条件才进入：

1. 12 kHz下电流带宽或延迟已被证明是系统瓶颈。
2. fast ISR最坏时间显著小于41.67 us周期。
3. 双SPI已完全异步解耦，不要求每个24 kHz周期完成pair。
4. ADC运放建立时间和采样窗口经示波器验证。
5. STM32F405 CPU与中断裕量足够。
6. 24 kHz下热、EMC和高调制度测试通过。

否则维持12 kHz，不以“频率更高”作为单独改造理由。

## 10. 测试矩阵

### 10.1 软件/主机测试

- 分频计数和每秒实际调用次数。
- tick计数回绕。
- 各模块`dt`正确性。
- 零阶保持年龄和超时边界。
- 新样本/旧样本/无效样本状态转换。
- SPI busy、CRC错误和pair丢失故障注入。
- Controller拆分前后定常输出一致性。
- 所有输入模式：PASSTHROUGH、VEL_RAMP、TORQUE_RAMP、POS_FILTER、TRAP_TRAJ、MIT。
- ADRC与摩擦补偿开关组合。
- 保存/恢复配置后频率参数和控制参数一致。

### 10.2 台架测试

1. 上电、校准、闭环、退出闭环循环100次。
2. 静止保持至少30分钟。
3. 正反向最低可控速度。
4. 低速换向与回差区穿越。
5. 速度阶跃：小信号、中信号、限幅附近。
6. 位置阶跃：多方向、多位置、多负载。
7. MIT刚度/阻尼阶跃。
8. 最大计划速度和最大计划加速度。
9. 低母线和高母线电压。
10. 冷机、热机和温升稳态。
11. 人为断开辅助编码器。
12. 人为制造主编码器CRC/超时。
13. CAN命令超时和心跳超时。
14. CPU压力：CAN高负载、USB日志、诊断读取同时开启。

### 10.3 仪器测量

逻辑分析仪/示波器至少观察：

- PWM定时器同步标记。
- ADC采样时刻。
- fast ISR入口/出口GPIO。
- MAIN_CS、AUX_CS、SPI_CLK。
- 栅极使能/故障引脚。
- 相电流或电流采样运放输出。

## 11. 验收标准

### 11.1 实时性

- `cl_deadline_miss_cnt == 0`。
- 连续30分钟最大负载运行无deadline miss。
- 12 kHz fast ISR最大时长不超过周期70%，目标不超过60%。
- 主样本和pair样本实际率达到目标并稳定。
- 无长期增长的pair busy；允许偶发但必须量化占比。

### 11.2 控制性能

- 电流阶跃不比基线明显恶化，超调满足项目限制。
- 稳态`Id`接近0，且高速不因相位陈旧显著升高。
- 速度/位置阶跃无新增持续振荡。
- 静止保持`Iq RMS`不高于基线，最好下降。
- 换向冲击不高于基线。
- MIT等效刚度和阻尼在允许误差内。

### 11.3 编码器

- 正常工况CRC错误率满足生产门限。
- 主辅时间偏差有实测上限。
- pair丢失时输出连续，不出现1/42圈分支跳变。
- 主编码器超龄时按设计安全停机。
- 辅助编码器恢复时按确认帧数重锁。

### 11.4 兼容性

- CAN常用命令和单位不变。
- NVM已有参数能够迁移或明确触发版本重置。
- FOC UI能显示新增诊断但旧操作不受影响。
- 原有生产测试脚本通过，并新增多速率专项测试。

## 12. 回滚方案

必须提供编译期配置：

```text
CONTROL_PROFILE_LEGACY_8K
CONTROL_PROFILE_MULTIRATE_8K
CONTROL_PROFILE_MULTIRATE_12K
```

建议不要长期维护三套完全不同代码，而是同一调度器通过常量配置频率；legacy profile仅用于A/B测试和紧急回退。

回滚触发条件：

- 任意deadline miss。
- 主编码器样本率不稳定。
- 高速`Id`明显增加。
- 电流/速度环出现新振荡。
- SPI busy或CRC错误显著增加。
- 温升或电磁噪声恶化。

## 13. 建议代码改造清单

预计涉及文件：

| 文件 | 主要改造 |
| --- | --- |
| `Firmware/Board/v3/Inc/main.h` | RCR和时基配置 |
| `Firmware/Board/v3/Inc/board.h` | 多速率常量、周期推导、static_assert |
| `Firmware/Board/v3/board.cpp` | ADC/PWM/ISR时序验证、GPIO测时 |
| `Firmware/MotorControl/main.cpp` | 多速率调度、信号保持、慢任务迁移 |
| `Firmware/MotorControl/odrive_main.h` | scheduler状态和诊断 |
| `Firmware/MotorControl/controller.cpp/.hpp` | 拆分位置、速度、转矩阶段；显式dt |
| `Firmware/MotorControl/encoder.cpp/.hpp` | main/pair独立序号、时间戳、快慢更新 |
| `Firmware/Drivers/MT6826S/mt6826s_spi.cpp/.hpp` | 采样计划、事务时间诊断、busy策略 |
| `Firmware/MotorControl/motor.cpp/.hpp` | 保持转矩命令、电流环显式dt |
| `Firmware/MotorControl/task_timer.hpp` | 总体和模块WCET统计 |
| `Firmware/MotorControl/debug_counters.hpp` | 新采样、超龄、分频和超时计数 |
| `Firmware/communication/can/can_simple.cpp` | 新增只读诊断项 |
| `Firmware/odrive-interface.yaml` | 新诊断接口定义，必要时NVM版本 |
| `Firmware/Tests/test_runner.cpp` | 调度、dt、保持和状态机单元测试 |
| `Firmware/Tests/hex_4342_mt6826s/` | 实机速率、SPI、阶跃和长稳脚本 |

## 14. 提交与评审建议

不要把所有阶段压成一个提交。建议提交顺序：

1. `test: capture 8 kHz timing and encoder baseline`
2. `refactor: introduce explicit multirate timebase and scheduler`
3. `refactor: split position and velocity controller stages`
4. `feat: add held torque command with age validation`
5. `feat: split MT6826S main and pair sampling schedules`
6. `diag: expose loop and SPI timing counters`
7. `feat: enable 12 kHz current loop profile`
8. `test: add multirate bench and fault-injection gates`

每个提交应能构建，关键中间提交应能运行或由编译期开关保持旧路径。

评审重点：

- 是否遗漏任何使用旧`current_meas_period`的状态更新。
- 未执行外环时转矩/位置命令是否被正确保持。
- 样本序号和时间戳是否原子一致。
- ISR内是否引入不可控执行时间。
- 错误恢复是否可能使用陈旧样本重新使能。
- 多速率改变是否偷偷改变了用户参数单位和实际增益。

## 15. 最终推荐

本项目的合理终态是：

```text
24 kHz PWM
12 kHz ADC + FOC current loop
12 kHz main encoder target / phase prediction every fast tick
6 kHz main+aux Vernier pair
2 kHz velocity / torque / ADRC / friction
1 kHz position / trajectory
```

先完成架构解耦，再提高电流频率；先保持239 Hz电流带宽，再根据实测决定是否提高到300～400 Hz。若12 kHz没有被证明是瓶颈，则不进入24 kHz电流环。
