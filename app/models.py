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


class Pose2D(BaseModel):
    x_cm: float = 0.0
    y_cm: float = 0.0
    heading_deg: float = 0.0


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
    encoder_direction_mode: str = ""
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


class LidarPoint(BaseModel):
    angle_deg: float
    distance_cm: float
    quality: float | None = None


class LidarStatus(BaseModel):
    auto_start: bool
    running: bool
    ready: bool
    state: str = "stopped"
    model: str = "RPLIDAR-A1"
    port: str = "/dev/ttyUSB0"
    scan_points: int = 0
    scan_hz: float = 0.0
    front_min_distance_cm: float | None = None
    left_min_distance_cm: float | None = None
    right_min_distance_cm: float | None = None
    overall_min_distance_cm: float | None = None
    last_scan_at: datetime | None = None
    last_error: str = ""


class LidarScan(BaseModel):
    points: list[LidarPoint] = Field(default_factory=list)
    captured_at: datetime | None = None


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


class OdometryStatus(BaseModel):
    running: bool
    ready: bool
    calibrated: bool
    stale: bool
    source: str = "encoder_dead_reckoning"
    pose: Pose2D = Field(default_factory=Pose2D)
    left_ticks: int | None = None
    right_ticks: int | None = None
    wheel_diameter_cm: float
    wheel_base_cm: float
    ticks_per_revolution: float
    linear_velocity_cm_s: float = 0.0
    angular_velocity_deg_s: float = 0.0
    distance_traveled_cm: float = 0.0
    correction_count: int = 0
    last_correction_at: datetime | None = None
    last_update_at: datetime | None = None
    last_error: str = ""


class ScanMatchResult(BaseModel):
    matched: bool
    confidence: float = 0.0
    pose: Pose2D = Field(default_factory=Pose2D)
    initial_pose: Pose2D = Field(default_factory=Pose2D)
    correction_x_cm: float = 0.0
    correction_y_cm: float = 0.0
    correction_heading_deg: float = 0.0
    points_used: int = 0
    hit_points: int = 0
    best_score: float = 0.0
    second_best_score: float = 0.0
    reason: str = ""


class LocalizationStatus(BaseModel):
    running: bool
    ready: bool
    state: str = "odometry_only"
    confidence: float = 0.0
    pose: Pose2D = Field(default_factory=Pose2D)
    last_match_confidence: float = 0.0
    last_match_at: datetime | None = None
    correction_count: int = 0
    distance_since_correction_cm: float = 0.0
    last_correction_x_cm: float = 0.0
    last_correction_y_cm: float = 0.0
    last_correction_heading_deg: float = 0.0
    map_loaded_from_disk: bool = False
    last_error: str = ""


class RelocalizeRequest(BaseModel):
    hint_pose: Pose2D | None = None
    search_xy_cm: float | None = Field(default=None, ge=5.0, le=300.0)
    search_heading_deg: float | None = Field(default=None, ge=3.0, le=90.0)


class MapCell(BaseModel):
    x_cm: float
    y_cm: float


class MapStatus(BaseModel):
    running: bool
    ready: bool
    resolution_cm: float
    size_cm: float
    map_id: str = ""
    revision: int = 0
    updates: int = 0
    free_cells: int = 0
    occupied_cells: int = 0
    dirty: bool = False
    learning_enabled: bool = True
    loaded_from_disk: bool = False
    persistence_path: str = ""
    last_saved_at: datetime | None = None
    last_loaded_at: datetime | None = None
    last_update_at: datetime | None = None
    last_error: str = ""


class MapSnapshot(BaseModel):
    resolution_cm: float
    size_cm: float
    map_id: str = ""
    revision: int = 0
    occupied: list[MapCell] = Field(default_factory=list)
    robot_pose: Pose2D = Field(default_factory=Pose2D)
    captured_at: datetime | None = None


class MapLearningRequest(BaseModel):
    enabled: bool


class MapPersistenceResult(BaseModel):
    success: bool
    action: str
    path: str
    cell_count: int = 0
    map_id: str = ""
    revision: int = 0
    reason: str = ""


class PlaceAnchor(BaseModel):
    anchor_id: str
    name: str = ""
    map_id: str
    map_revision: int = 0
    pose: Pose2D
    descriptor: list[float] = Field(default_factory=list)
    valid_sectors: int = 0
    created_at: datetime


class PlaceRecognitionResult(BaseModel):
    recognized: bool
    anchor_id: str = ""
    anchor_name: str = ""
    similarity: float = 0.0
    second_similarity: float = 0.0
    margin: float = 0.0
    estimated_pose: Pose2D = Field(default_factory=Pose2D)
    heading_offset_deg: float = 0.0
    verified: bool = False
    scan_match: ScanMatchResult | None = None
    reason: str = ""


class PlaceRecognitionStatus(BaseModel):
    ready: bool
    anchor_count: int = 0
    map_id: str = ""
    persistence_path: str = ""
    last_recognition_at: datetime | None = None
    last_similarity: float = 0.0
    last_anchor_id: str = ""
    last_error: str = ""


class PlaceAnchorCreate(BaseModel):
    name: str = Field(default="", max_length=100)


class MapIdentity(BaseModel):
    map_id: str
    revision: int


class NavigationGoal(BaseModel):
    x_cm: float
    y_cm: float
    heading_deg: float | None = None


class NavigationPlan(BaseModel):
    found: bool
    reason: str = ""
    goal: NavigationGoal
    waypoints: list[Pose2D] = Field(default_factory=list)
    planned_distance_cm: float = 0.0
    cells_explored: int = 0
    uses_unknown_space: bool = False


class NavigationStatus(BaseModel):
    enabled: bool
    hardware_execution_allowed: bool
    running: bool
    state: str = "idle"
    reason: str = ""
    goal: NavigationGoal | None = None
    waypoint_index: int = 0
    waypoint_count: int = 0
    planned_distance_cm: float = 0.0
    distance_remaining_cm: float | None = None
    localization_confidence: float | None = None
    started_at: datetime | None = None
    last_update_at: datetime | None = None
    last_error: str = ""
    recovery_available: bool = False


class DockStatus(BaseModel):
    configured: bool
    dock_pose: Pose2D | None = None
    approach_goal: NavigationGoal | None = None
    distance_to_dock_cm: float | None = None
    map_id: str = ""
    map_revision: int = 0
    persistent: bool = False
    valid_for_current_map: bool = False
    reason: str = ""


class SimulationSensors(BaseModel):
    front_distance_cm: float | None = Field(default=None, ge=0, le=10000)
    left_distance_cm: float | None = Field(default=None, ge=0, le=10000)
    right_distance_cm: float | None = Field(default=None, ge=0, le=10000)
    front_bumper_left: bool = False
    front_bumper_right: bool = False
    lidar_connected: bool = False
    lidar_min_distance_cm: float | None = Field(default=None, ge=0, le=10000)


class SimulationMapObstacle(BaseModel):
    x_cm: float
    y_cm: float
    radius_cm: float = Field(default=20.0, ge=5.0, le=200.0)
