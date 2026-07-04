"""Smoke test: connect to the running backend's /ws, receive the first
status broadcast, print it. Run while the backend is up:
    .venv\\Scripts\\python.exe -m foc_backend._ws_smoke
"""
import asyncio
import json
import sys

import websockets


async def main():
    uri = 'ws://127.0.0.1:8000/ws'
    async with websockets.connect(uri) as ws:
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=3.0)
        except asyncio.TimeoutError:
            print('TIMEOUT: no message within 3s', file=sys.stderr)
            return
        msg = json.loads(raw)
        print('first message:', json.dumps(msg, ensure_ascii=False))
        # Expect a status broadcast on connect.
        assert msg.get('type') == 'status', f'expected status, got {msg.get("type")}'
        assert msg['connected'] is False
        print('OK: status handshake works')


if __name__ == '__main__':
    asyncio.run(main())
