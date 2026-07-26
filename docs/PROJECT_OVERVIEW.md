# ODrive 单轴关节驱动器项目报告

> 基于 ODrive v3.6、STM32F405、DRV8301、双 MT6826S Vernier 编码器  
> 固件分支：`feature/mt6826s-encoder`  
> CAN 协议版本：v1.9 (`0x00000109`)

本文档面向固件维护、上位机对接和机器人关节电机调试。项目目标是将 ODrive v3.6 裁剪为单轴关节驱动器固件，以 CAN 作为主要外部接口，支持输出轴绝对位置估计、位置/速度/力矩/MIT 模式、摩擦补偿、ADRC 扰动补偿和生产配置持久化。

---

## 1. 系统概览

```text
                  CAN 1 Mbps
                      |
        +-------------v--------------+
        |      CAN Simple + Ext      |
        +-------------+--------------+
                      |
        +-------------v--------------+
        | Controller                 |
        | position / velocity / MIT  |
        | ADRC + friction + limits   |
        +-------------+--------------+
                      |
        +-------------v--------------+
        | Motor FOC + SVPWM          |
        +-------------+--------------+
                      |
        +-------------v--------------+
        | Dual MT6826S Vernier       |
        | output-shaft estimation    |
        +----------------------------+
```

当前报告只描述产品使用的 CAN 主链路。

## 2. 硬件规格

| 项目 | 参数 | 说明 |
| --- | --- | --- |
| 主控 | STM32F405RGT6 | 168 MHz Cortex-M4 |
| 栅极驱动 | DRV8301 | 三相 BLDC 驱动 |
| 编码器 | 双 MT6826S | SPI 绝对磁编码器 |
| 编码器分辨率 | 32768 CPR × 2 | 单个 MT6826S 15 bit |
| 输出轴估计 | Vernier 游标合成 | main=42, aux=41 |
| 减速比 | 42:1 / 41:1 | 两路编码器相差 1 齿 |
| 电机类型 | HIGH_CURRENT BLDC | 极对数 14 |
| 制动电阻 | 2.0 ohm | 已启用 |
| 母线电压范围 | 14 V 到 65 V | 欠压 14 V，过压 65 V |
| 逆变器温度保护 | 100 到 120 degC | 降额区间 |

## 3. 固件结构

```text
Firmware/
├── Board/v3/                         ODrive v3.6 板级支持
├── MotorControl/
│   ├── axis.cpp                      轴状态机
│   ├── controller.cpp                位置/速度/MIT 控制，ADRC，摩擦补偿
│   ├── encoder.cpp                   MT6826S Vernier 合成和校准
│   ├── motor.cpp                     FOC、电机模型、电流限制
│   └── trap_traj.cpp                 梯形轨迹规划
├── communication/can/
│   ├── can_simple.cpp                CAN Simple 命令处理
│   ├── CAN_PROTOCOL_PRODUCTION.md    协议冻结说明
│   └── CAN_PROTOCOL_MANUAL.md        厂商手册风格 CAN 协议
├── Drivers/MT6826S/                  编码器底层驱动
└── production_config.h               单型号生产配置
```

核心裁剪方向：

- 单轴固件，只暴露 Axis0。
- CAN 为主要产品通信接口。
- 电机、编码器、母线、电阻、温度等硬件身份参数在 `production_config.h` 固化。
- NVM 只保存可调参数，例如电流限制、控制增益、Vernier offset、轨迹限制、摩擦补偿、ADRC 配置。

## 4. 控制策略

### 4.1 控制链路

每个 FOC 电流环周期调用一次 `Controller::update()`。正常位置/速度模式的力矩合成可概括为：

```text
input command
  -> trajectory / ramp / passthrough
  -> position loop
  -> velocity loop
  -> ADRC trim
  -> friction compensation
  -> torque/current/velocity limits
  -> motor-side torque command
```

位置模式下的主要计算关系：

```text
pos_err = pos_setpoint - pos_estimate
pos_err = deadband(pos_err)

pos_integrator_vel += pos_integrator_gain * dt * pos_err
vel_des = vel_setpoint + pos_gain * pos_err + pos_integrator_vel
vel_des = clamp(vel_des, +/- vel_limit)

vel_err = vel_des - vel_estimate
torque = torque_setpoint
       + vel_gain * vel_err
       + vel_integrator_torque
       + adrc_trim
       + friction_compensation

torque = clamp(torque, +/- torque_limit)
```

### 4.2 ADRC 扰动补偿

ADRC 使用三阶扩展状态观测器估计等效扰动，并以受限力矩 trim 的形式叠加到控制输出。

```text
e  = z1 - pos_estimate
z1 = z1 + dt * (z2 - beta1 * e)
z2 = z2 + dt * (z3 - beta2 * e + b0 * last_torque)
z3 = z3 + dt * (-beta3 * e)

adrc_trim = clamp(-z3 / b0, +/- adrc_trim_torque_limit)
```

配置项：

| Item | 名称 | 说明 |
| --- | --- | --- |
| `0x6D` | `adrc_trim_slew_rate` | ADRC 输出斜率限制，Nm/s |
| `0x6E` | `enable_adrc` | ADRC 开关 |
| `0x6F` | `adrc_trim_torque_limit` | ADRC 输出上限，Nm |

### 4.3 摩擦补偿

摩擦补偿采用静摩擦、库伦摩擦、Stribeck 衰减和粘滞项组合。方向优先使用期望速度；低速或静止时使用位置误差方向；无明确运动意图时补偿缓降到零。

```text
mag = Tc + (Ts - Tc) * exp(-(vel / vs)^2)
comp = direction * mag + B * vel
comp = clamp(comp, +/- friction_max_torque)
comp = slew_limit(comp, friction_torque_slew_rate)
```

摩擦补偿分两类使能：

| Item | 名称 | 作用模式 |
| --- | --- | --- |
| `0x70` | `enable_friction_compensation` | 位置/速度伺服 |
| `0x7C` | `enable_mit_friction_compensation` | MIT 实时阻抗 |

### 4.4 模式矩阵

| 功能 | TORQUE+PASSTHROUGH | VELOCITY | PROFILE POSITION | MIT |
| --- | :---: | :---: | :---: | :---: |
| 位置 P/I | - | - | yes | - |
| 速度 P/I | - | yes | yes | - |
| 梯形轨迹 | - | - | yes | - |
| MIT PD | - | - | - | yes |
| ADRC trim | - | yes | yes | yes |
| 摩擦补偿 | - | yes | yes | yes, 独立开关 |
| 速度限制 | torque-mode gate | yes | yes | torque-mode gate |
| 力矩钳位 | yes | yes | yes | yes |

## 5. CAN 通信协议

完整协议手册见 [CAN_PROTOCOL_MANUAL.md](../Firmware/communication/can/CAN_PROTOCOL_MANUAL.md)。本节只保留报告中的关键摘要。

### 5.1 基本参数

| 项目 | 值 |
| --- | --- |
| CAN 类型 | Classic CAN 2.0 |
| 仲裁 ID | 标准 11-bit ID |
| 默认波特率 | 1 Mbps |
| 默认节点 ID | 0 |
| ID 编码 | `(node_id << 5) | cmd_id` |
| 普通帧字节序 | little-endian |
| MIT 帧字节序 | AK/T-Motor 风格 big-endian bit packing |
| 协议版本 | v1.9 (`0x00000109`) |

### 5.2 常用命令

| Cmd | 名称 | 方向 | DLC | 说明 |
| --- | --- | --- | ---: | --- |
| `0x001` | Heartbeat | 驱动器 -> 主站 | 8 | 状态、错误、运行 flags |
| `0x007` | Set_Axis_State | 主站 -> 驱动器 | 4 | 请求 IDLE/CLOSED_LOOP 等状态 |
| `0x009` | Get_Encoder_Estimates | 驱动器 -> 主站 | 8 | 位置 turn、速度 turn/s |
| `0x00B` | Set_Controller_Mode | 主站 -> 驱动器 | 8 | 设置 control/input mode |
| `0x00C` | Set_Input_Pos | 主站 -> 驱动器 | 8 | 位置指令，turn |
| `0x00D` | Set_Input_Vel | 主站 -> 驱动器 | 8 | 速度指令，turn/s |
| `0x00E` | Set_Input_Torque | 主站 -> 驱动器 | 4 | 力矩指令，Nm |
| `0x00F` | Set_Limits | 主站 -> 驱动器 | 8 | 速度限制 turn/s，电流限制 A |
| `0x011` | Set_Traj_Vel_Limit | 主站 -> 驱动器 | 4 | 位置规划速度限制 |
| `0x012` | Set_Traj_Accel_Limits | 主站 -> 驱动器 | 8 | 位置规划加减速度 |
| `0x014` | Get_Iq | 驱动器 -> 主站 | 8 | Iq setpoint / measured |
| `0x017` | Get_Bus_Voltage_Current | 驱动器 -> 主站 | 8 | 母线电压/电流 |
| `0x018` | Clear_Errors | 主站 -> 驱动器 | 0/8 | 清错 |
| `0x01E` | Extended_Command | 双向 | 8 | 配置、诊断、标定 |
| `0x01F` | Set_MIT_Control | 主站 -> 驱动器 | 8 | MIT 实时阻抗帧 |

### 5.3 扩展命令

`0x01E Extended_Command` 的通用布局：

```text
request:
byte0    sub_cmd
byte1    item
byte2    type, read=0
byte3    reserved
byte4..7 value

response:
byte0    sub_cmd
byte1    item
byte2    status
byte3    type/flags
byte4..7 value
```

常用 subcommand：

| SubCmd | 名称 | 说明 |
| --- | --- | --- |
| `0x01` | Get_Axis_Status_Ex | 读取状态、ready flags、axis_error |
| `0x03` | Save_Configuration | 保存 NVM，ACK 后复位 |
| `0x05` | Get_Device_Info | 协议版本、固件版本、序列号 |
| `0x06/0x07` | Get/Set_Basic_Config | 电流限制、编码器 offset、控制增益 |
| `0x0A` | Get_Vernier_Diagnostics | 输出轴估计和诊断 |
| `0x0B/0x0C` | Get/Set_Control_Config | 轨迹限制、看门狗、ADRC、摩擦补偿 |
| `0x0D` | Vernier_Calibration | 多点静态 Vernier offset 标定 |
| `0x0E` | Get_Fault_Snapshot | 故障/超速快照 |

### 5.4 MIT 控制帧

MIT 帧仅在 `TORQUE_CONTROL + INPUT_MODE_MIT` 下参与实时控制。普通位置/速度单位是 turn，但 MIT 帧使用 rad。

| 字段 | 位宽 | 范围 | 单位 |
| --- | ---: | --- | --- |
| `p_des` | 16 | -12.5 .. +12.5 | rad |
| `v_des` | 12 | -45 .. +45 | rad/s |
| `kp` | 12 | 0 .. 500 | Nm/rad |
| `kd` | 12 | 0 .. 5 | Nm/(rad/s) |
| `t_ff` | 12 | -18 .. +18 | Nm |

```text
byte0: p[15:8]
byte1: p[7:0]
byte2: v[11:4]
byte3: v[3:0] | kp[11:8]
byte4: kp[7:0]
byte5: kd[11:4]
byte6: kd[3:0] | t[11:8]
byte7: t[7:0]
```

MIT 帧需要持续发送，建议 50 到 200 Hz，调试默认 100 Hz。停止前应先发送 neutral MIT 帧，再切回 IDLE。

## 6. 标定与调试流程

### 6.1 推荐上电流程

```text
1. 读取 Heartbeat，确认 axis_error=0
2. 如有错误，发送 Clear_Errors
3. 确认 motor_calibrated 和 encoder_ready
4. 设置业务模式：
   - PROFILE_POSITION: control=3, input=5
   - VELOCITY:         control=2, input=2
   - TORQUE:           control=1, input=1
   - MIT:              control=1, input=9
5. 设置安全限制：速度、电流、轨迹速度/加速度
6. 请求 CLOSED_LOOP_CONTROL
7. 周期发送对应模式的运动命令
```

### 6.2 Vernier 多点标定

Vernier 几何 offset 标定使用扩展命令 `0x0D`，推荐自动流程：

```text
reset points
-> 慢速扫过多个输出轴位置
-> 每个静止点 capture
-> fit aux offset
-> apply
-> 验证位置/速度估计
-> save configuration
```

应用 fit 只修改 RAM 配置；确认效果后必须执行 `Save_Configuration` 才能持久化。

### 6.3 摩擦参数标定

摩擦补偿参数由双向 breakaway torque 测试估计：

```text
1. 进入 TORQUE_CONTROL + PASSTHROUGH
2. 正反方向慢速增加输出轴扭矩
3. 检测输出轴速度超过阈值时的 breakaway torque
4. 根据 breakaway torque 生成 static/coulomb/max/slew 参数
5. 写入 0x74..0x7B
6. 分别启用 0x70 或 0x7C
7. 保存配置
```

当前实测静摩擦/库伦摩擦量级约为 `0.15..0.4 Nm`，因此 ADRC trim 上限若只有 `0.005 Nm` 基本只能作为微扰补偿；需要承担明显负载扰动时应提高 `adrc_trim_torque_limit`。

## 7. 测试结果摘要

### 7.1 位置静态误差

测试脚本：[test_position_steps.py](../Firmware/Tests/hex_4342_mt6826s/test_position_steps.py)

| 项目 | 值 |
| --- | --- |
| 控制模式 | POSITION_CONTROL + TRAP_TRAJ |
| 目标序列 | 0, 180, 360, 540, 720, 1080 deg |
| 速度/加速度限制 | 60 rpm / 60 rpm/s |
| 每目标保持 | 2 s |
| 反馈源 | Vernier 输出轴估计 |

代表性结果：

| Target | Mean Err | RMS Err | Max Abs Err |
| ---: | ---: | ---: | ---: |
| 0 deg | -0.0017 deg | 0.0018 deg | 0.0026 deg |
| 180 deg | 0.0029 deg | 0.0029 deg | 0.0029 deg |
| 360 deg | 0.0025 deg | 0.0026 deg | 0.0040 deg |
| 1080 deg | 0.0033 deg | 0.0034 deg | 0.0046 deg |

结论：低速到位后的静态精度很好，没有明显多圈累计误差；主要限制来自磁编码器噪声、Vernier 合成噪声和机械回差。

### 7.2 速度阶梯响应

测试脚本：[test_velocity_steps.py](../Firmware/Tests/hex_4342_mt6826s/test_velocity_steps.py)

| Ref | Mean | RMS Err |
| ---: | ---: | ---: |
| 0.1 rpm | 0.131 rpm | 0.057 rpm |
| 1 rpm | 0.966 rpm | 0.063 rpm |
| 10 rpm | 9.919 rpm | 0.285 rpm |
| 20 rpm | 19.885 rpm | 0.400 rpm |
| 40 rpm | 39.836 rpm | 1.112 rpm |
| 60 rpm | 59.798 rpm | 1.116 rpm |

结论：速度链路单位已经统一为输出轴 `turn/s`；高速段误差和波动增加，主要是动态带宽、Vernier 速度估计噪声和摩擦补偿共同影响。

### 7.3 MIT 曲线跟踪

测试脚本：[run_mit_curve_test.py](../Firmware/Tests/hex_4342_mt6826s/run_mit_curve_test.py)

| 项目 | 值 |
| --- | --- |
| 控制模式 | TORQUE_CONTROL + MIT |
| 帧率 | 100 Hz |
| 轨迹 | 五阶 minimum-jerk |
| 路径点 | 0, 40, 360, 120, 20, -360, 0 deg |
| 每段时间 | 4 s |
| kp / kd | 2.0 / 0.2 |

结果：

| 指标 | 值 |
| --- | ---: |
| Mean tracking error | 0.7600 deg |
| RMS tracking error | 5.6589 deg |
| Max absolute error | 13.0264 deg |

结论：MIT 协议链路、单位和方向基本正确。误差随速度显著增加，属于动态滞后问题，不是固定 offset。若要提升 MIT 动态精度，应提高 `kp/kd`、启用 MIT 摩擦补偿，并根据电流限制评估是否需要更高输出力矩余量。

## 8. 生产配置摘要

| 参数 | 值 | 备注 |
| --- | --- | --- |
| CAN 波特率 | 1 Mbps | 默认 |
| CAN 节点 ID | 0 | 默认 |
| 电机类型 | HIGH_CURRENT | BLDC |
| 极对数 | 14 | 固化 |
| 默认电流限制 | 5.0 A | NVM 可调 |
| 校准电流 | 2.0 A | 固化 |
| 校准电压上限 | 3.0 V | 固化 |
| 编码器模式 | SPI_ABS_MT6826S_VERNIER | 固化 |
| 编码器 CPR | 32768 | 固化 |
| Vernier main ratio | 42.0 | 输出轴合成 |
| Vernier aux ratio | 41.0 | 输出轴合成 |
| 制动电阻 | 2.0 ohm | 已启用 |
| 母线欠压/过压 | 14 V / 65 V | 固化 |

## 9. 当前结论与后续建议

已经完成并验证的部分：

- CAN 协议主链路和扩展配置链路可用。
- MT6826S Vernier 输出轴位置估计可用于高精度静态定位。
- 位置模式静态误差进入 0.005 deg 量级。
- 速度模式在 60 rpm 输出轴速度下能稳定跟踪。
- MIT 模式可以按平滑曲线运行，动态误差主要由带宽/摩擦/力矩裕量决定。
- NVM 持久化、摩擦补偿、ADRC、MIT 摩擦补偿已通过 CAN 配置项暴露。

建议继续优化：

- 将 CAN 协议文档和上位机协议表保持为单一来源，避免 item 漂移。
- 为 MIT 曲线测试增加自动参数扫描，输出 `kp/kd` 与 RMS 误差关系。
- 对摩擦补偿参数建立自动复测流程，区分静摩擦、库伦摩擦和速度估计噪声。
- 进一步检查高速度下 Vernier 速度估计噪声对速度环和 MIT 阻尼项的影响。

## 10. 文件索引

| 路径 | 内容 |
| --- | --- |
| `Firmware/production_config.h` | 单型号生产配置 |
| `Firmware/MotorControl/controller.cpp` | 控制器核心 |
| `Firmware/MotorControl/controller.hpp` | 控制器配置和状态声明 |
| `Firmware/MotorControl/encoder.cpp` | 编码器校准和 Vernier 合成 |
| `Firmware/MotorControl/motor.cpp` | 电机控制、FOC、电流限制 |
| `Firmware/communication/can/can_simple.cpp` | CAN 命令处理 |
| `Firmware/communication/can/CAN_PROTOCOL_PRODUCTION.md` | 协议冻结说明 |
| `Firmware/communication/can/CAN_PROTOCOL_MANUAL.md` | 厂商手册风格 CAN 协议 |
| `foc_ui/backend/odrive_can/protocol.py` | 上位机 CAN 编解码 |
| `Firmware/Tests/hex_4342_mt6826s/test_position_steps.py` | 位置静态误差测试 |
| `Firmware/Tests/hex_4342_mt6826s/test_velocity_steps.py` | 速度阶梯测试 |
| `Firmware/Tests/hex_4342_mt6826s/run_mit_curve_test.py` | MIT 曲线跟踪测试 |

