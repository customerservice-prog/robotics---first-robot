from fastapi.testclient import TestClient

from app.main import app


def test_health_and_status():
    with TestClient(app) as client:
        health = client.get('/health').json()
        assert health['ok'] is True
        assert health['voice']['running'] is False
        assert health['camera']['running'] is False
        assert health['lidar']['running'] is False
        assert 'localization' in health
        assert 'spatial' in health
        assert 'odometry' in health
        assert 'map' in health
        assert 'navigation' in health
        assert 'dock' in health
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



def test_odometry_map_and_navigation_endpoints():
    with TestClient(app) as client:
        reset = client.post(
            '/api/odometry/reset',
            json={'x_cm': 0, 'y_cm': 0, 'heading_deg': 0},
        )
        assert reset.status_code == 200
        assert reset.json()['ready'] is True
        assert reset.json()['calibrated'] is True

        client.post('/api/map/clear')
        obstacle = client.post(
            '/api/simulation/map-obstacle',
            json={'x_cm': 100, 'y_cm': 0, 'radius_cm': 20},
        )
        assert obstacle.status_code == 200
        assert obstacle.json()['occupied_cells'] > 0

        snapshot = client.get('/api/map/snapshot')
        assert snapshot.status_code == 200
        assert snapshot.json()['occupied']

        plan = client.post(
            '/api/navigation/plan',
            json={'x_cm': 200, 'y_cm': 0},
        )
        assert plan.status_code == 200
        assert plan.json()['found'] is True
        assert any(abs(point['y_cm']) > 20 for point in plan.json()['waypoints'])

        nav = client.get('/api/navigation/status')
        assert nav.status_code == 200
        assert nav.json()['enabled'] is True


def test_operator_stop_cancels_simulated_navigation():
    with TestClient(app) as client:
        client.post('/api/map/clear')
        client.post(
            '/api/odometry/reset',
            json={'x_cm': 0, 'y_cm': 0, 'heading_deg': 0},
        )
        started = client.post(
            '/api/navigation/start',
            json={'x_cm': 60, 'y_cm': 0},
        )
        assert started.status_code == 200
        assert started.json()['running'] is True

        stopped = client.post('/api/stop')
        assert stopped.status_code == 200
        nav = client.get('/api/navigation/status').json()
        assert nav['running'] is False
        assert nav['state'] == 'cancelled'



def test_recovery_replan_and_dock_foundation():
    with TestClient(app) as client:
        client.post('/api/map/clear')
        client.post(
            '/api/odometry/reset',
            json={'x_cm': 0, 'y_cm': 0, 'heading_deg': 0},
        )

        dock = client.post('/api/dock/set-current')
        assert dock.status_code == 200
        dock_data = dock.json()
        assert dock_data['configured'] is True
        assert dock_data['approach_goal']['x_cm'] == -60
        assert dock_data['approach_goal']['y_cm'] == 0

        dock_plan = client.post('/api/dock/plan-return')
        assert dock_plan.status_code == 200
        assert dock_plan.json()['found'] is True

        client.post(
            '/api/navigation/plan',
            json={'x_cm': 100, 'y_cm': 40},
        )
        replanned = client.post('/api/navigation/replan')
        assert replanned.status_code == 200
        assert replanned.json()['found'] is True
        nav = client.get('/api/navigation/status').json()
        assert nav['state'] == 'replanned'
        assert nav['running'] is False



def test_localization_and_map_mode_endpoints():
    with TestClient(app) as client:
        loc = client.get('/api/localization/status')
        assert loc.status_code == 200
        assert 'confidence' in loc.json()

        frozen = client.post('/api/map/learning', json={'enabled': False})
        assert frozen.status_code == 200
        assert frozen.json()['learning_enabled'] is False

        resumed = client.post('/api/map/learning', json={'enabled': True})
        assert resumed.status_code == 200
        assert resumed.json()['learning_enabled'] is True
