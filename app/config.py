from functools import lru_cache
from pathlib import Path

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_prefix="RIBITICS_", env_file=".env", extra="ignore")

    name: str = "Ribitics"
    mode: str = "simulation"
    serial_port: str = "/dev/ttyACM0"
    serial_baud: int = 115200
    database_path: str = "data/ribitics.db"
    ollama_url: str = "http://127.0.0.1:11434"
    ollama_model: str = "qwen2.5:3b"

    enable_local_tts: bool = False
    tts_command: str = "espeak-ng"
    enable_voice_loop: bool = False
    voice_engine: str = "vosk"
    wake_phrase: str = "hey ribitics"
    vosk_model_path: str = "models/vosk-model-small-en-us-0.15"
    microphone_device: str | None = None
    voice_sample_rate: int = 16000
    voice_block_size: int = 4000
    voice_command_timeout_seconds: float = 8.0

    enable_camera: bool = False
    camera_device: str = "0"
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: float = 12.0
    camera_jpeg_quality: int = 80
    camera_motion_threshold: float = 12.0

    enable_lidar: bool = False
    lidar_model: str = "RPLIDAR-A1"
    lidar_port: str = "/dev/ttyUSB0"
    lidar_forward_angle_deg: float = 0.0
    lidar_front_arc_deg: float = 35.0
    lidar_max_distance_mm: float = 6000.0

    obstacle_stop_cm: float = 35.0
    obstacle_warn_cm: float = 70.0
    require_proximity_for_forward: bool = False

    enable_odometry: bool = True
    odometry_calibrated: bool = False
    wheel_diameter_cm: float = 6.5
    wheel_base_cm: float = 15.0
    encoder_ticks_per_revolution: float = 360.0
    odometry_poll_hz: float = 20.0
    odometry_stale_seconds: float = 1.0
    simulation_encoder_ticks_per_second: float = 360.0

    enable_mapping: bool = True
    map_resolution_cm: float = 5.0
    map_size_cm: float = 1200.0
    map_robot_radius_cm: float = 20.0
    mapping_poll_hz: float = 5.0

    enable_supervised_navigation: bool = False
    navigation_allow_unknown: bool = False
    navigation_max_goal_distance_cm: float = 500.0
    navigation_max_linear: float = 0.22
    navigation_max_angular: float = 0.28
    navigation_waypoint_tolerance_cm: float = 12.0
    navigation_heading_tolerance_deg: float = 12.0
    navigation_timeout_seconds: float = 120.0

    dock_approach_distance_cm: float = 60.0

    max_motor_percent: int = 65
    control_token: str = "change-me-before-network-use"

    def ensure_data_dir(self) -> None:
        Path(self.database_path).parent.mkdir(parents=True, exist_ok=True)


@lru_cache
def get_settings() -> Settings:
    settings = Settings()
    settings.ensure_data_dir()
    return settings
