# ODrive M0 ABZ CAN 测试脚本

这个目录放单驱 M0 的 CANSimple 硬件测试脚本，默认用于：

- ABZ 增量编码器电机
- 极对数 `10`
- 1024 线 ABZ，也就是 ODrive CPR `4096`
- PCAN-USB 通道 `PCAN_USBBUS1`
- CAN 波特率 `250000`
- CAN node id `0`

运行前请确认电机机械端安全，校准和闭环测试会让电机通电，速度环测试会让电机低速转动。

## 依赖

在当前 Python 环境中需要有 `python-can`：

```powershell
python -m pip show python-can
```

如果没有：

```powershell
python -m pip install python-can
```

## 1. 校准 M0 ABZ

在仓库根目录 `E:\project_source_code\ODrive` 下运行：

```powershell
python .\Firmware\Tests\can_m0_abz\calibrate_m0_abz.py
```

脚本会执行：

1. 等待 M0 heartbeat。
2. 写入基础配置：
   - `motor_type = HIGH_CURRENT`
   - `pole_pairs = 10`
   - `encoder_mode = INCREMENTAL`
   - `encoder_cpr = 4096`
   - `calibration_current = 5A`
   - `current_lim = 10A`
3. 清错误。
4. 请求 `AXIS_STATE_FULL_CALIBRATION_SEQUENCE`。
5. 监控 heartbeat，等待回到 `IDLE`。
6. 读取校准结果：
   - `phase_resistance`
   - `phase_inductance`
   - `phase_offset`
   - `direction`
7. 设置运行态 `motor/encoder pre_calibrated` 标志。

默认不保存配置。如果希望校准后保存到 ODrive，并触发重启：

```powershell
python .\Firmware\Tests\can_m0_abz\calibrate_m0_abz.py --save
```

注意：当前固件中 ABZ incremental 编码器重启后可能不会仅凭保存的 `pre_calibrated` 自动变成 `encoder_ready=True`。如果重启后速度环脚本提示未 ready，可以使用速度环脚本的 `--set-precalibrated-if-needed` 参数。

常用参数：

```powershell
python .\Firmware\Tests\can_m0_abz\calibrate_m0_abz.py --channel PCAN_USBBUS1 --bitrate 250000 --node-id 0 --pole-pairs 10 --encoder-cpr 4096
```

## 2. 校准后速度环短测

先确保已经完成校准，然后运行：

```powershell
python .\Firmware\Tests\can_m0_abz\velocity_loop_test.py
```

脚本会执行：

1. 等待 heartbeat。
2. 检查 `motor_calibrated=True` 且 `encoder_ready=True`。
3. 设置速度控制：
   - `control_mode = VELOCITY_CONTROL`
   - `input_mode = PASSTHROUGH`
   - 默认速度限制 `20 turns/s`
   - 默认电流限制 `5A`
4. 请求 `CLOSED_LOOP_CONTROL`。
5. 默认发送 `0.05 turns/s`，持续 `2s`。
6. 周期读取：
   - encoder position / velocity
   - Iq setpoint / measured
   - heartbeat error flags
7. 发送 `input_vel = 0` 并回到 `IDLE`。

如果 ODrive 重启后运行态 ready 标志没有恢复，可以运行：

```powershell
python .\Firmware\Tests\can_m0_abz\velocity_loop_test.py --set-precalibrated-if-needed
```

如果想改测试速度和时间：

```powershell
python .\Firmware\Tests\can_m0_abz\velocity_loop_test.py --velocity 0.05 --duration 2.0
```

如果测试中报错，希望脚本结束时自动清错误：

```powershell
python .\Firmware\Tests\can_m0_abz\velocity_loop_test.py --clear-at-end
```

## 建议测试顺序

```powershell
python .\tools\verify_pcan_odrive.py
python .\Firmware\Tests\can_m0_abz\calibrate_m0_abz.py
python .\Firmware\Tests\can_m0_abz\velocity_loop_test.py --set-precalibrated-if-needed
```

如果三步都通过，说明：

- PCAN 到 ODrive CAN 链路正常。
- M0 ABZ 基础配置正常。
- 电机校准和编码器 offset 校准正常。
- 校准后能进入速度环并完成低速闭环测试。

## 3. MIT 模式测试

MIT 模式使用 `0x01F Set MIT Control`，也就是 AK/TMotor 常见的 8 字节压缩帧：

```text
p_des  16 bit, rad,      -12.5 ~ 12.5
v_des  12 bit, rad/s,    -45   ~ 45
kp     12 bit, Nm/rad,    0    ~ 500
kd     12 bit, Nm/(rad/s),0    ~ 5
t_ff   12 bit, Nm,       -18   ~ 18
```

MIT 模式需要持续发送命令帧。原因是 MIT 帧本身就是实时控制输入，主控应以固定周期刷新目标位置、速度、kp、kd 和前馈力矩。脚本默认周期是 `20ms`，也就是约 `50Hz`。实际项目中可以根据总线负载和控制需求提高到 `100Hz` 或更高。

测试脚本默认只发送中性 MIT 帧，验证固件能进入 `TORQUE_CONTROL + INPUT_MODE_MIT` 并保持无错误：

```powershell
python .\Firmware\Tests\can_m0_abz\mit_mode_test.py --set-precalibrated-if-needed
```

脚本流程：

1. 检查 `motor_calibrated=True` 和 `encoder_ready=True`。
2. 设置：
   - `control_mode = TORQUE_CONTROL`
   - `input_mode = MIT`
3. 在 `IDLE` 状态先发送若干中性 MIT 帧，避免闭环刚启动时使用旧 MIT 输入。
4. 请求 `CLOSED_LOOP_CONTROL`，同时继续发送 MIT 帧。
5. 持续发送 MIT 帧并监控 encoder、Iq 和 heartbeat 错误。
6. 返回 `IDLE`。

如果要测试一个很小的阻尼/保持帧，可以加小增益：

```powershell
python .\Firmware\Tests\can_m0_abz\mit_mode_test.py --set-precalibrated-if-needed --hold-kp 0.2 --hold-kd 0.02
```

如果要测试前馈力矩，务必从很小值开始，例如：

```powershell
python .\Firmware\Tests\can_m0_abz\mit_mode_test.py --set-precalibrated-if-needed --torque-ff 0.005
```

注意：标准 12-bit MIT torque 在 `-18 ~ 18Nm` 范围内没有精确的 0，脚本的中性帧会在相邻的两个 raw 值之间交替发送，让平均力矩尽量接近 0。

## 4. 齿槽转矩校准

齿槽校准需要烧录包含 `0x08 GET_ANTICOGGING_STATUS` 和 `0x09 SET_ANTICOGGING_CONFIG` 的新固件。脚本会通过 CAN 做这些事：

1. 检查 `motor_calibrated=True` 和 `encoder_ready=True`。
2. 可选地把 encoder linear count 归零。
3. 设置齿槽校准阈值：
   - `calib_pos_threshold`，单位是 encoder counts。
   - `calib_vel_threshold`，单位是 encoder counts/s。
4. 重置齿槽校准 index。
5. 进入位置闭环。
6. 发送 `START_ANTICOGGING`。
7. 周期读取固件内部 `anticogging.index` 和 `anticogging_valid`，直到 index 跑完并返回完成。

建议先用 3505-KV650 这组保守参数重新做基础校准：

```powershell
python .\Firmware\Tests\can_m0_abz\calibrate_m0_abz.py --pole-pairs 10 --encoder-cpr 4096 --kv 650 --calibration-current 3 --current-limit 8
```

然后跑齿槽校准：

```powershell
python .\Firmware\Tests\can_m0_abz\anticogging_calibration.py --set-precalibrated-if-needed --zero-linear-count --max-duration 900
```

如果 index 走得很慢，可以适当放宽阈值，例如：

```powershell
python .\Firmware\Tests\can_m0_abz\anticogging_calibration.py --set-precalibrated-if-needed --zero-linear-count --calib-pos-threshold 20 --calib-vel-threshold 300 --max-duration 900
```

如果电机抖动明显，先降低位置/速度环增益：

```powershell
python .\Firmware\Tests\can_m0_abz\anticogging_calibration.py --set-precalibrated-if-needed --zero-linear-count --pos-gain 3 --vel-gain 0.01 --vel-integrator-gain 0.03 --max-duration 900
```

完成后脚本会打印 `PASS: anticogging calibration was triggered and completed...`。当前 CAN 扩展可以确认校准完成，但是否保存齿槽表仍取决于后续是否执行保存配置。

## 5. 速度环/位置环参数调节

`tune_control_gains.py` 可以通过 CAN 写入并测试：

- `pos_gain`
- `vel_gain`
- `vel_integrator_gain`

只写参数、不让电机动：

```powershell
python .\Firmware\Tests\can_m0_abz\tune_control_gains.py --set-precalibrated-if-needed --pos-gain 1 --vel-gain 0.005 --vel-integrator-gain 0.01
```

测试速度环：

```powershell
python .\Firmware\Tests\can_m0_abz\tune_control_gains.py --set-precalibrated-if-needed --mode velocity --velocity 0.1 --duration 2 --pos-gain 1 --vel-gain 0.005 --vel-integrator-gain 0.01
```

如果希望速度指令不要一步跳到目标值，可以加 `--ramp-time`，单位是秒：

```powershell
python .\Firmware\Tests\can_m0_abz\tune_control_gains.py --set-precalibrated-if-needed --clear-at-start --mode velocity --velocity 2.0 --ramp-time 3 --duration 8 --pos-gain 2 --vel-gain 0.015 --vel-integrator-gain 0.008 --clear-at-end
```

当前这台 3505-KV650、12V 供电、齿槽补偿已开启的实测结果：

- `pos_gain=2.0`、`vel_gain=0.015`、`vel_integrator_gain=0.008` 下，`2.0 turns/s` 可以稳定完成短测。
- `2.2 turns/s`、`2.5 turns/s`、`3.0 turns/s` 即使加速度斜坡，也会在接近目标速度后触发 `odrv.error=0x8`，对应母线回灌电流错误。继续提高速度前建议先处理供电回灌能力、刹车电阻或相关保护参数。

测试位置环小阶跃：

```powershell
python .\Firmware\Tests\can_m0_abz\tune_control_gains.py --set-precalibrated-if-needed --mode position --step 0.02 --duration 2 --pos-gain 1 --vel-gain 0.005 --vel-integrator-gain 0.01
```

如果确认一组参数可用，可以保存。保存会触发 ODrive 重启：

```powershell
python .\Firmware\Tests\can_m0_abz\tune_control_gains.py --set-precalibrated-if-needed --pos-gain 1 --vel-gain 0.005 --vel-integrator-gain 0.01 --save
```

调参建议从小到大：

1. 先把 `vel_integrator_gain` 设小，甚至接近 0，避免积分项慢慢堆大。
2. 先调 `vel_gain`，让速度响应能跟上但不抖。
3. 再调 `pos_gain`，让位置阶跃能收敛且不过冲。
4. 最后少量增加 `vel_integrator_gain`，改善静差。
