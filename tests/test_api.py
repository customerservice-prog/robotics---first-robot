from fastapi.testclient import TestClient

from app.main import app


def test_health_and_status():
    with TestClient(app) as client:
        assert client.get('/health').json()['ok'] is True
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
