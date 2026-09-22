from datetime import datetime

from pydantic import BaseModel, Field


class DriveCommand(BaseModel):
    linear: float = Field(ge=-1.0, le=1.0)
    angular: float = Field(ge=-1.0, le=1.0)


class ChatRequest(BaseModel):
    message: str = Field(min_length=1, max_length=4000)


class ChatResponse(BaseModel):
    reply: str
    remembered: bool = False
    model: str = "fallback"


class MemoryCreate(BaseModel):
    content: str = Field(min_length=1, max_length=4000)
    tags: list[str] = Field(default_factory=list)
    importance: int = Field(default=5, ge=1, le=10)


class MemoryRecord(BaseModel):
    id: int
    content: str
    tags: list[str]
    importance: int
    created_at: datetime


class RobotStatus(BaseModel):
    name: str
    mode: str
    connected: bool
    stopped: bool
    estop: bool
    left_motor: int
    right_motor: int
    battery_voltage: float | None = None
    left_ticks: int | None = None
    right_ticks: int | None = None
    front_distance_cm: float | None = None
    left_distance_cm: float | None = None
    right_distance_cm: float | None = None
    front_bumper_left: bool = False
    front_bumper_right: bool = False
    lidar_connected: bool = False
    lidar_min_distance_cm: float | None = None
    last_message: str = ""


class VoiceStatus(BaseModel):
    auto_start: bool
    running: bool
    ready: bool
    state: str = "stopped"
    engine: str = "vosk"
    wake_phrase: str = "hey ribitics"
    model_path: str = ""
    last_heard: str = ""
    last_reply: str = ""
    last_error: str = ""


class CameraStatus(BaseModel):
    auto_start: bool
    running: bool
    ready: bool
    state: str = "stopped"
    device: str = "0"
    width: int = 0
    height: int = 0
    fps: float = 0.0
    frames_captured: int = 0
    motion_detected: bool = False
    motion_score: float = 0.0
    last_frame_at: datetime | None = None
    last_error: str = ""


class SpatialStatus(BaseModel):
    sensors_available: bool
    clear_to_move_forward: bool
    hazard_level: str
    reason: str
    obstacle_stop_cm: float
    obstacle_warn_cm: float
    front_distance_cm: float | None = None
    left_distance_cm: float | None = None
    right_distance_cm: float | None = None
    nearest_forward_distance_cm: float | None = None
    front_bumper_left: bool = False
    front_bumper_right: bool = False
    lidar_connected: bool = False
    lidar_min_distance_cm: float | None = None


class SimulationSensors(BaseModel):
    front_distance_cm: float | None = Field(default=None, ge=0, le=10000)
    left_distance_cm: float | None = Field(default=None, ge=0, le=10000)
    right_distance_cm: float | None = Field(default=None, ge=0, le=10000)
    front_bumper_left: bool = False
    front_bumper_right: bool = False
    lidar_connected: bool = False
    lidar_min_distance_cm: float | None = Field(default=None, ge=0, le=10000)
