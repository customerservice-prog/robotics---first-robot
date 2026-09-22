from fastapi.testclient import TestClient

from app.main import app


def test_health_and_status():
    with TestClient(app) as client:
        health = client.get('/health').json()
        assert health['ok'] is True
        assert health['voice']['running'] is False
        status = client.get('/api/status').json()
        assert status['connected'] is True
        assert status['mode'] == 'simulation'


def test_drive_and_stop():
    with TestClient(app) as client:
        moved = client.post('/api/drive', json={'linear': 1, 'angular': 0})
        assert moved.status_code == 200
        assert moved.json()['left_motor'] > 0
        stopped = client.post('/api/stop')
        assert stopped.json()['left_motor'] == 0
        assert stopped.json()['right_motor'] == 0


def test_voice_status_endpoint():
    with TestClient(app) as client:
        status = client.get('/api/voice/status')
        assert status.status_code == 200
        assert status.json()['engine'] == 'vosk'
        assert status.json()['wake_phrase'] == 'hey ribitics'
