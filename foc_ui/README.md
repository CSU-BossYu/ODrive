# FOC 单驱上位机 (foc_ui)

为 STM32F405 + DRV8301 + AS5047P 单驱 FOC 工程（`F:\souce_code\foc`）配套的
本地调试上位机：浏览器 UI + 本地 Python 服务，实时显示 16 通道波形、
控制电机启停、调参、查看日志、录制数据。

```
┌──────────────┐   USB CDC   ┌─────────────────┐   WebSocket   ┌──────────────┐
│  STM32 F405  │ ──────────► │ Python FastAPI  │ ────────────► │   Browser    │
│  FOC 固件    │  二进制流   │  (本地服务)     │   60 Hz       │  Vue + uPlot │
│              │ ◄────────── │                 │ ◄──────────── │              │
└──────────────┘  ASCII CLI  └─────────────────┘   命令下发    └──────────────┘
```

---

## 1. 商业许可证清单（零风险）

本工程**不使用任何 GPL/LGPL/AGPL 等"传染性"许可证的依赖**。所有运行时和
构建期依赖均为 MIT 或 BSD-3-Clause，可安全用于闭源商业产品，唯一义务是
保留版权声明。

### 1.1 后端 Python 依赖（`backend/requirements.txt`）

| 包 | 版本 | 许可证 | 用途 |
|---|---|---|---|
| fastapi | 0.115.6 | **MIT** | Web 框架 + WebSocket |
| uvicorn[standard] | 0.34.0 | **BSD-3-Clause** | ASGI 服务器 |
| pyserial | 3.5 | **BSD-3-Clause** | 串口通信（Windows 友好） |
| pydantic | 2.10.4 | **MIT** | 请求/响应模型 |
| websockets | 14.1 | **BSD-3-Clause** | WS 协议（uvicorn 依赖） |
| Python 解释器 | 3.10+ | PSF (BSD-like) | — |

### 1.2 前端依赖（`frontend/package.json`）

| 包 | 版本 | 许可证 | 用途 |
|---|---|---|---|
| vue | ^3.5.13 | **MIT** | UI 框架 |
| uplot | ^1.6.31 | **MIT** | 高性能 Canvas 波形 |
| vite | ^6.0.5 | **MIT** | 构建工具（仅开发期） |
| @vitejs/plugin-vue | ^5.2.1 | **MIT** | Vue SFC 编译（仅开发期） |
| typescript | ^5.7.2 | **Apache-2.0** | 类型检查（仅开发期） |
| vue-tsc | ^2.2.0 | **MIT** | Vue 类型检查（仅开发期） |

### 1.3 关于 esbuild 的 npm audit 警告

`npm audit` 会报 3 个 high severity 漏洞，全部位于 **esbuild**——它只是
**开发期打包工具**，不会进入 `dist/` 生产产物。漏洞描述是"通过
`NPM_CONFIG_REGISTRY` 环境变量劫持影响 Deno 模块"，对本项目（本地桌面
工具）无实际威胁。**不要 `npm audit fix --force`**，那会升级到 vite@8
引入破坏性变更。

### 1.4 义务提示

MIT/BSD 许可证的唯一义务是**在分发时保留版权声明和许可证文本**。如果你
把上位机打包分发，需要：
- 在"关于"或文档里列出上述依赖及许可证（一份 NOTICE 文件足够）
- 不要把作者的版权声明删掉

本工程未使用任何需要开源自身代码、或需要支付商业 License 费用的依赖。

---

## 2. 快速开始

### 2.1 前置条件
- **Python 3.10+**（推荐 3.12）
- **Node.js 20+**（仅开发/重新构建前端需要；纯使用已构建产物不需要）
- **STM32 固件已烧录阶段 0 的帧率修复版本**（见第 4 节）

### 2.2 一键启动（推荐）

双击 `run.bat`，或在命令行：
```cmd
cd F:\souce_code\foc_ui
run.bat
```
首次运行会自动创建 `backend\.venv` 并装依赖。然后浏览器打开
`http://127.0.0.1:8000`。

### 2.3 手动启动

```cmd
REM 1) 后端
cd F:\souce_code\foc_ui\backend
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe -m foc_backend.main

REM 2) 前端（仅开发模式需要单独跑；生产模式后端会托管 dist/）
cd F:\souce_code\foc_ui\frontend
npm install
npm run build        REM 一次性构建，产物在 dist/
REM 或: npm run dev  REM 热重载，浏览器开 http://127.0.0.1:5173
```

---

## 3. 使用说明

### 3.1 连接

1. 用 USB 线接好 STM32（注意：是 USB Device 口，不是 ST-Link 口）
2. 浏览器打开 `http://127.0.0.1:8000`
3. 顶部状态栏选择 COM 端口（描述里通常含 "STMicroelectronics Virtual COM Port"）
4. 点"连接"
5. 在右侧参数面板把 **遥测帧率** 改成 100（或更高），波形开始流动

### 3.2 波形

- 默认显示 6 个精选通道：`vel_ref` / `vel_fb` / `iq_ref` / `iq_meas` / `id_meas` / `vbus`
- 底部色块可勾选其余 10 个通道
- 鼠标拖拽缩放，双击复位
- 时间窗口约 10-20 秒（取决于帧率和缓冲长度）

### 3.3 控制

左侧面板：
- **电流环/速度环/位置环**：启动对应模式
- **MIT 模式**：启动 CAN MIT 阻抗模式（需 CAN 主机持续发命令，500ms 超时会自动停）
- **停止**：立即停机（中性 PWM）
- **清错**：清除可清除的错误（FATAL 位需断电）
- **MIT 注入器**：无 CAN 硬件时测试 MIT 模式（`test mit` 命令）

### 3.4 参数

右侧面板的 9 个参数对应固件 `set` 命令，输入框自带范围校验：
| 参数 | 范围 | 单位 |
|---|---|---|
| vel_limit | 0..60 | rad/s |
| ilimit | 0..3 | A |
| ramp | 0.1..100 | rad/s² |
| vel_kp / vel_ki | ≥0 | — |
| pos_kp | 0..1000 | — |
| flux | 0..0.05 | Wb |
| watchdog | 0..60 | s（0=关） |
| stream_hz | 0..500 | Hz（0=关） |

点 **dump** 按钮读取固件当前值。

### 3.5 日志 / CLI

右下方：
- 顶部色块是快捷命令（status/dump/errors/gate/diag/help/stop/clear）
- 中间是日志流（按 tag 着色：FOC 灰/FAULT 红/BOOT 蓝/CLI 亮蓝/CAN 黄）
- 底部输入框可手发任意 CLI 命令（Enter 发送，↑↓ 历史）
- 滚动日志会自动暂停跟随，点"↓ 跟随"恢复

### 3.6 录制

顶部状态栏的 **● 录制** 按钮：
- 点开始：自动以时间戳命名 CSV（`foc_capture_YYYY-MM-DDTHH-MM-SS.csv`）
- 点停止：关闭文件
- CSV 列：`t_ms, seq, pos, vel_ref, ..., state_flags`（17 列）
- 可用 Python/MATLAB/Excel 打开分析

---

## 4. 配套固件改动（阶段 0）

本上位机依赖固件侧的**遥测帧率修复**。原固件 `telemetryTask` 用
`osDelay(1000)` 唤醒，导致 `set stream_hz` 实际只能输出 ~1Hz。修复后改动
3 处（已编译通过，0 Error 0 Warning）：

| 文件 | 改动 |
|---|---|
| `Core/Src/freertos.c` | telemetryTask `osDelay(1000)` → `osDelay(2)`；状态快照统一锁内读取，CAN/stream 发送使用 `_from(&st)` 版本避免无锁重读 |
| `Users/Motor/motor_can.c/.h` | `send_feedback` 拆成 `send_feedback()` + `send_feedback_from(st)`；后者加 50Hz 内部时间戳节流 |
| `Users/Motor/motor_stream.c/.h` | `poll()` 拆成 `poll()` + `emit(st)`；emit 接收外部快照 |

详见 `F:\souce_code\foc\AI_HANDOFF_V2.md` 第 13 节。

### 烧录验证清单
1. `set stream_hz 100` → 串口助手 hex 模式应每 10ms 看到 `55 AA ...` 36 字节帧
2. seq 连续递增（0,1,2,...,255,0）
3. CAN MIT 模式反馈帧约 50Hz（不是 500Hz）
4. 空载低速验证无异常

---

## 5. 工程结构

```
foc_ui/
├─ run.bat                         一键启动脚本
├─ README.md                       本文件
├─ backend/                        Python FastAPI 后端
│  ├─ requirements.txt             依赖锁定（全 MIT/BSD）
│  ├─ .venv/                       虚拟环境（首次运行自动创建）
│  └─ foc_backend/
│     ├─ __init__.py
│     ├─ main.py                   FastAPI 入口 + 路由 + WS 命令分发
│     ├─ telemetry.py              二进制帧定义 + CRC8 + 16通道表（真相源）
│     ├─ serial_link.py            串口线程 + 3路解复用状态机
│     ├─ cli_responder.py          CLI 命令串行化 + 流式响应收集
│     ├─ ws_hub.py                 WebSocket 广播（60Hz 降采样）
│     ├─ recorder.py               CSV 录制
│     ├─ _selftest.py              解析链路自测（无需硬件）
│     └─ _ws_smoke.py              WS 握手 smoke test
└─ frontend/                       Vue 3 + Vite + uPlot 前端
   ├─ package.json
   ├─ vite.config.ts               开发代理 /api 和 /ws 到后端
   ├─ tsconfig.json
   ├─ index.html
   ├─ dist/                        构建产物（npm run build 生成）
   └─ src/
      ├─ main.ts                   Vue 入口
      ├─ App.vue                   主布局（三栏）
      ├─ style.css                 全局样式（暗色主题）
      ├─ types.ts                  TS 类型 + WS 消息契约
      ├─ channels.ts               16通道表 + 错误位表 + 配色
      ├─ composables/
      │  └─ useFocSocket.ts        单 WS 连接 + 自动重连 + 响应式状态
      └─ components/
         ├─ WaveformPanel.vue      uPlot 16通道波形 + 通道开关
         ├─ ControlPanel.vue       启停/模式切换/MIT注入
         ├─ ParamPanel.vue         9 个 set 参数 UI 化
         ├─ LogConsole.vue         日志流 + CLI 输入（历史/快捷）
         └─ StatusBar.vue          连接/电压/电流/错误/丢包/录制
```

---

## 6. 关键设计决策

### 6.1 为什么选 Web 方案（浏览器 + 本地 Python 服务），不用 PyQt/Electron？

- **零许可证风险**：PyQt 是 GPL（商用必须开源或买 License），PySide6 是
  LGPL（静态编译就违规）。Web 方案全用 MIT/BSD。
- **零打包复杂度**：Electron 体积 100+ MB，Tauri 需要 Rust 工具链。本地
  服务 + 浏览器 = 一个 Python 进程，用户已装浏览器。
- **前后端解耦**：串口/解析在 Python（生态成熟），UI 在浏览器（渲染快）。

### 6.2 为什么 WebSocket 单连接 + 消息类型字段，而不是多端点？

- 浏览器管理一个连接的生命周期最简单（重连、错误处理）
- 消息按 `type` 字段路由，前端一个 composable 统一处理
- 后端 `WsHub` 用一个 asyncio Queue 派发，避免多端点的并发复杂度

### 6.3 为什么后端做 60Hz 降采样？

固件可输出 100-500Hz 二进制帧，但浏览器 DOM 渲染上限约 60Hz（显示器刷新率）。
后端 `WsHub.broadcast_telemetry` 对每个客户端独立限流到 60Hz，避免无谓的
网络/CPU 消耗。前端 uPlot 在每帧增量更新，60Hz 下完全流畅。

### 6.4 为什么遥测帧错误位只看低 16 位？

固件 `motor_stream.c` 把 28 位错误码截断到 int16 传输（`errors & 0xFFFF`）。
OVERSPEED/STALL/CONFIG 等高位错误在波形里看不到。完整 28 位错误码需通过
`errors` CLI 命令或 `status` 字段获取。StatusBar 组件已对低 16 位做完整
解码显示，足够日常调试。

---

## 7. 已知限制

1. **单连接**：后端设计为单浏览器标签页使用。多标签页会共享同一个串口
   连接，状态可能不一致。
2. **CLI 响应靠超时收尾**：固件 CLI 无请求 ID/echo，后端用 150ms 无新行
   判定响应结束。极慢的响应可能被截断。
3. **遥测快照非严格原子**：固件 `motor_axis_get_status` 在锁内拷贝，但
   20kHz ISR 仍可能在中途更新某些字段。单帧值可能有微小不一致，对调试
   可接受。
4. **MIT 模式需持续注入**：固件 CAN 看门狗 500ms 超时，USB 注入或真实
   CAN 主机必须持续发命令，否则自动停。
5. **错误位截断**：见 6.4。

---

## 8. 故障排查

| 现象 | 可能原因 | 解决 |
|---|---|---|
| 连接后无波形 | 遥测流未开 | 参数面板设 stream_hz=100 |
| COM 端口列表空 | 驱动未装 / USB 未插 | 检查设备管理器，装 STM32 CDC 驱动 |
| 波形只有 1Hz | 固件未烧录阶段 0 修复 | 重新编译烧录固件 |
| 大量 CRC 错误 | USB 线质量差 / 干扰 | 换短粗 USB 线，避免 USB Hub |
| CLI 命令无响应 | 后端未连串口 | 顶部状态栏先点"连接" |
| 前端白屏 | dist 未构建 | `run.bat build` 或 `npm run build` |

---

## 9. 开发自测命令

```cmd
REM 后端解析链路自测（无需硬件）
cd F:\souce_code\foc_ui\backend
.venv\Scripts\python.exe -m foc_backend._selftest

REM WS 握手 smoke test（需后端在跑）
.venv\Scripts\python.exe -m foc_backend._ws_smoke

REM 前端类型检查 + 构建
cd F:\souce_code\foc_ui\frontend
npm run build
```
