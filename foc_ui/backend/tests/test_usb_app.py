"""FastAPI USB routes, SPA fallback and independent CAN/USB status coverage."""

from pathlib import Path
import sys

from fastapi.testclient import TestClient

sys.path.insert(0, str(Path(__file__).parents[1]))

from odrive_can.app import app  # noqa: E402
from odrive_usb.app import USB_STATE  # noqa: E402
from odrive_usb.replay import decode_bundle  # noqa: E402
from odrive_usb.transport import FakeSerialTransport  # noqa: E402


def test_usb_api_does_not_replace_can_routes_or_docs():
    with TestClient(app) as client:
        schema = client.get('/api/usb/schema')
        assert schema.status_code == 200
        assert schema.json()['protocol_release'] == '4.5'
        assert len(schema.json()['channels']) >= 15
        assert client.get('/api/can/status').status_code == 200
        assert client.get('/docs').status_code == 200
        assert client.get('/api/not-a-route').status_code == 404


def test_static_index_and_spa_history_fallback():
    with TestClient(app) as client:
        root = client.get('/')
        history = client.get('/diagnostics/session/42')
        if (Path(__file__).parents[2] / 'frontend' / 'dist' / 'index.html').is_file():
            assert root.status_code == 200
            assert '<div id="app"></div>' in root.text
            assert history.status_code == 200
            assert '<div id="app"></div>' in history.text
        else:
            assert root.status_code == 200
            assert history.status_code == 404


def test_fake_usb_connect_disconnect_has_no_control_side_effect():
    with TestClient(app) as client:
        connected = client.post('/api/usb/connect', json={'fake': True})
        assert connected.status_code == 200
        status = client.get('/api/usb/status').json()
        assert status['connected'] is True
        disconnected = client.post('/api/usb/disconnect')
        assert disconnected.status_code == 200
        assert disconnected.json()['was_connected'] is True
        assert not USB_STATE.transport.status.connected


def test_real_usb_connect_retries_one_transient_first_write(monkeypatch):
    original = USB_STATE.transport.connect
    attempts = 0

    def flaky_connect(*, port=None, serial_device=None):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise PermissionError("transient WriteFile failure")
        return original(port=port, serial_device=FakeSerialTransport())

    monkeypatch.setattr(USB_STATE.transport, "connect", flaky_connect)
    with TestClient(app) as client:
        response = client.post('/api/usb/connect', json={
            'port': 'COM4', 'fake': False})
        assert response.status_code == 200
        assert attempts == 2
        client.post('/api/usb/disconnect')


def test_motion_session_requires_explicit_hardware_estop():
    with TestClient(app) as client:
        response = client.post('/api/usb/session/begin', json={
            'motion': True, 'hardware_estop_confirmed': False})
        assert response.status_code == 409
        assert 'hardware estop' in response.json()['error']


def test_usb_scope_arm_accepts_manual_action_and_raw_export():
    with TestClient(app) as client:
        assert client.post('/api/usb/connect', json={'fake': True}).status_code == 200
        armed = client.post('/api/usb/scope/arm', json={'arm': 1})
        manual = client.post('/api/usb/scope/arm', json={'arm': 2})
        assert armed.status_code == 200 and armed.json()['arm'] == 1
        assert manual.status_code == 200 and manual.json()['arm'] == 2
        raw = client.get('/api/usb/export/raw')
        assert raw.status_code == 200
        assert raw.headers['content-type'].startswith('application/octet-stream')
        client.post('/api/usb/disconnect')


def test_usb_replay_export_is_versioned_and_decodable():
    with TestClient(app) as client:
        replay = client.get('/api/usb/export/replay')
        assert replay.status_code == 200
        assert replay.headers['content-type'].startswith(
            'application/vnd.odrive.usb-replay+json')
        bundle = decode_bundle(replay.content)
        assert bundle['format'] == 'odrive-usb-replay'
        assert bundle['schema_version'] == 1


def test_usb_websocket_has_own_status_stream():
    with TestClient(app) as client:
        with client.websocket_connect('/ws/usb') as websocket:
            message = websocket.receive_json()
            assert message['type'] == 'usb_status'
