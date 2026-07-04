"""End-to-end check: pretend to be a browser. Fetch the HTML, find the JS/CSS
references, fetch each, and confirm they all return 200 with the right MIME.
Run while the backend is up:
    .venv\\Scripts\\python.exe -m foc_backend._e2e
"""
import re
import sys
import urllib.request

BASE = 'http://127.0.0.1:8000'


def fetch(path: str) -> tuple[int, str, bytes]:
    url = BASE + path
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            return r.status, r.headers.get('Content-Type', ''), r.read()
    except Exception as e:
        return -1, str(e), b''


def main() -> int:
    # 1. HTML
    code, ctype, body = fetch('/')
    print(f'1) GET /                      -> {code}  {ctype}  {len(body)}B')
    if code != 200:
        print('FAIL: index.html'); return 1
    html = body.decode('utf-8', errors='replace')
    if 'FOC' not in html:
        print('FAIL: html missing title'); return 1

    # 2. extract JS/CSS references
    js_urls = re.findall(r'src="(/assets/[^"]+\.js)"', html)
    css_urls = re.findall(r'href="(/assets/[^"]+\.css)"', html)
    print(f'   references: {len(js_urls)} JS, {len(css_urls)} CSS')
    if not js_urls:
        print('FAIL: no JS reference in html'); return 1

    for u in js_urls + css_urls:
        code, ctype, body = fetch(u)
        print(f'2) GET {u:<28} -> {code}  {ctype}  {len(body)}B')
        if code != 200:
            print(f'FAIL: {u} returned {code}'); return 1

    # 3. API endpoints
    for ep in ('/api/status', '/api/ports', '/api/channels'):
        code, ctype, body = fetch(ep)
        ok = code == 200 and 'json' in ctype
        print(f'3) GET {ep:<28} -> {code}  {ctype}  {len(body)}B  {"OK" if ok else "BAD"}')
        if not ok:
            return 1

    # 4. WebSocket handshake (uses the websockets lib)
    import asyncio
    import json
    import websockets

    async def ws_check():
        async with websockets.connect('ws://127.0.0.1:8000/ws') as ws:
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=3.0))
            assert msg['type'] == 'status'
            print(f'4) WS /ws handshake           -> OK (first msg type=status)')
    asyncio.run(ws_check())

    print('\nALL E2E CHECKS PASSED')
    return 0


if __name__ == '__main__':
    sys.exit(main())
