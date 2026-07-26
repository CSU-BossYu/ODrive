# FOC CAN 上位机

这是当前单轴 ODrive 固件的 CAN-only 调试界面。通信路径为：

`STM32 bxCAN (1 Mbit/s Classic CAN) -> PCAN/python-can -> FastAPI WebSocket -> Vue`

USB 仅用于固件日志输出，不参与上位机控制。项目不再包含串口二进制流、ASCII CLI、Fibre 或 odrivetool 支持。

## 启动

Windows 下运行：

```bat
run.bat dev
run.bat build
run.bat
```

- `dev`：后端监听 8000，Vite 开发服务器监听 5173。
- `build`：构建前端后启动后端。
- 无参数：直接提供已构建的 `frontend/dist`。

后端也可单独启动：

```powershell
cd backend
python -m odrive_can.app
```

## 主要接口

- `GET /api/can/interfaces`：列出 python-can 接口。
- `POST /api/can/connect`：连接 CAN，节点号范围 0..63。
- `POST /api/can/disconnect`：断开 CAN。
- `GET /api/can/status`：适配器、设备心跳及队列状态。
- `WS /ws/can`：遥测、控制、参数和标定消息。
- `GET /docs`：FastAPI 自动生成的接口文档。

协议以 `Firmware/communication/can/CAN_PROTOCOL_PRODUCTION.md` 为准。上位机要求协议版本 `0x0000010A`。

## 验证

```powershell
cd backend
python -m pytest -q

cd ../frontend
npm run build
```

运行依赖见 `backend/requirements.txt`，当前核心依赖为 FastAPI、Uvicorn、Pydantic、WebSockets 和 python-can。
