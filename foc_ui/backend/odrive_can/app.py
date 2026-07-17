"""CAN-only FastAPI application for the FOC upper computer."""

from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from .main import CAN_STATE, can_disconnect, can_router

logger = logging.getLogger('odrive_can.app')
FRONTEND_DIST = Path(__file__).resolve().parents[2] / 'frontend' / 'dist'


@asynccontextmanager
async def lifespan(_: FastAPI):
    CAN_STATE.loop = asyncio.get_running_loop()
    logger.info('FOC CAN backend up at http://127.0.0.1:8000')
    try:
        yield
    finally:
        await can_disconnect()
        logger.info('FOC CAN backend down')


app = FastAPI(title='FOC CAN upper-computer', lifespan=lifespan)
app.include_router(can_router)


@app.get('/', response_class=HTMLResponse)
async def index():
    index_path = FRONTEND_DIST / 'index.html'
    if index_path.is_file():
        return HTMLResponse(index_path.read_text(encoding='utf-8'))
    return HTMLResponse(
        '<h2>FOC CAN backend is running</h2>'
        '<p>Build the frontend with <code>npm run build</code>, or use dev mode.</p>'
        '<p><a href="/docs">CAN API documentation</a></p>'
    )


for directory in ('assets', 'static'):
    static_path = FRONTEND_DIST / directory
    if static_path.is_dir():
        app.mount(f'/{directory}', StaticFiles(directory=static_path),
                  name=f'frontend-{directory}')


def main():
    import uvicorn
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(name)s %(levelname)s %(message)s',
    )
    uvicorn.run(app, host='127.0.0.1', port=8000, log_level='info')


if __name__ == '__main__':
    main()
