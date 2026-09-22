from fastapi.testclient import TestClient

from app.main import app


def test_health_and_status():
    with TestClient(app) as client:
        health = client.get('/health').json()
        assert health['ok'] is True
        assert health['voice']['running'] is False
        assert health['camera']['running'] is False
        assert health['lidar']['running'] is False
        assert 'spatial' in health
        status = client.get('/api/status').json()
        assert status['connected'] is True
        assert status['mode'] == 'simulation'


def test_drive_and_stop():
    with TestClient(app) as client:
        client.post('/api/simulation/sensors', json={'front_distance_cm': None})
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


def test_camera_status_endpoint_does_not_require_camera_package():
    with TestClient(app) as client:
        status = client.get('/api/camera/status')
        assert status.status_code == 200
        assert status.json()['state'] == 'stopped'
        assert status.json()['ready'] is False


def test_lidar_status_and_empty_scan_do_not_require_lidar_package():
    with TestClient(app) as client:
        status = client.get('/api/lidar/status')
        assert status.status_code == 200
        assert status.json()['state'] == 'stopped'
        scan = client.get('/api/lidar/scan')
        assert scan.status_code == 200
        assert scan.json()['points'] == []


def test_simulated_obstacle_blocks_forward_but_allows_reverse():
    with TestClient(app) as client:
        try:
            spatial = client.post(
                '/api/simulation/sensors',
                json={'front_distance_cm': 20},
            )
            assert spatial.status_code == 200
            assert spatial.json()['clear_to_move_forward'] is False
            assert spatial.json()['hazard_level'] == 'stop'

            blocked = client.post('/api/drive', json={'linear': 1, 'angular': 0})
            assert blocked.status_code == 409
            assert 'Obstacle' in blocked.json()['detail']

            reverse = client.post('/api/drive', json={'linear': -1, 'angular': 0})
            assert reverse.status_code == 200
            assert reverse.json()['left_motor'] < 0
        finally:
            client.post('/api/stop')
            client.post('/api/simulation/sensors', json={'front_distance_cm': None})
