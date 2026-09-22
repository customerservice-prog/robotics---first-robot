from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import Depends, FastAPI, Header, HTTPException, Query
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles

from app.config import get_settings
from app.hardware import SimulatedHardware
from app.models import (
    CameraStatus,
    ChatRequest,
    ChatResponse,
    DockStatus,
    DriveCommand,
    LidarScan,
    LidarStatus,
    LocalizationStatus,
    MapLearningRequest,
    MapPersistenceResult,
    MapSnapshot,
    MapStatus,
    MemoryCreate,
    MemoryRecord,
    NavigationGoal,
    NavigationPlan,
    NavigationStatus,
    OdometryStatus,
    Pose2D,
    RelocalizeRequest,
    RobotStatus,
    SimulationMapObstacle,
    ScanMatchResult,
    SimulationSensors,
    SpatialStatus,
    VoiceStatus,
)
from app.navigation import (
    AStarPlanner,
    CorrelativeLocalizer,
    DifferentialOdometry,
    DockingFoundation,
    LocalOccupancyMap,
    NavigationError,
    SupervisedNavigator,
)
from app.perception.camera import CameraService
from app.perception.lidar import LidarService
from app.perception.spatial import SpatialAwareness
from app.robot import RobotController, UnsafeDriveError
from app.services.conversation import ConversationService
from app.services.llm import LocalLLM
from app.services.memory import MemoryStore
from app.services.speech import LocalSpeaker
from app.services.voice import OfflineVoiceAssistant

settings = get_settings()
memory = MemoryStore(settings.database_path)
llm = LocalLLM(settings.ollama_url, settings.ollama_model, settings.name)

if settings.mode.lower() == "esp32":
    from app.hardware.esp32_serial import ESP32SerialHardware

    hardware = ESP32SerialHardware(settings.name, settings.serial_port, settings.serial_baud)
else:
    hardware = SimulatedHardware(
        settings.name,
        ticks_per_second_at_full_power=settings.simulation_encoder_ticks_per_second,
    )

camera = CameraService(
    auto_start=settings.enable_camera,
    device=settings.camera_device,
    width=settings.camera_width,
    height=settings.camera_height,
    fps=settings.camera_fps,
    jpeg_quality=settings.camera_jpeg_quality,
    motion_threshold=settings.camera_motion_threshold,
)
lidar = LidarService(
    auto_start=settings.enable_lidar,
    model=settings.lidar_model,
    port=settings.lidar_port,
    forward_angle_deg=settings.lidar_forward_angle_deg,
    front_arc_deg=settings.lidar_front_arc_deg,
    max_distance_mm=settings.lidar_max_distance_mm,
)
awareness = SpatialAwareness(
    hardware,
    stop_cm=settings.obstacle_stop_cm,
    warn_cm=settings.obstacle_warn_cm,
    require_proximity_for_forward=settings.require_proximity_for_forward,
    lidar=lidar,
)
odometry = DifferentialOdometry(
    hardware,
    wheel_diameter_cm=settings.wheel_diameter_cm,
    wheel_base_cm=settings.wheel_base_cm,
    ticks_per_revolution=settings.encoder_ticks_per_revolution,
    poll_hz=settings.odometry_poll_hz,
    stale_seconds=settings.odometry_stale_seconds,
    calibrated=settings.odometry_calibrated,
)
occupancy_map = LocalOccupancyMap(
    lidar,
    odometry,
    resolution_cm=settings.map_resolution_cm,
    size_cm=settings.map_size_cm,
    robot_radius_cm=settings.map_robot_radius_cm,
    poll_hz=settings.mapping_poll_hz,
    persistence_path=settings.map_persistence_path,
)
planner = AStarPlanner(
    occupancy_map,
    robot_radius_cm=settings.map_robot_radius_cm,
    allow_unknown=settings.navigation_allow_unknown,
)
localizer = CorrelativeLocalizer(
    occupancy_map,
    lidar,
    odometry,
    poll_hz=settings.localization_poll_hz,
    search_xy_cm=settings.localization_search_xy_cm,
    search_heading_deg=settings.localization_search_heading_deg,
    xy_step_cm=settings.localization_xy_step_cm,
    heading_step_deg=settings.localization_heading_step_deg,
    min_points=settings.localization_min_points,
    min_confidence=settings.localization_min_confidence,
    max_correction_cm=settings.localization_max_correction_cm,
    max_correction_deg=settings.localization_max_correction_deg,
    confidence_decay_distance_cm=settings.localization_confidence_decay_distance_cm,
    confidence_decay_seconds=settings.localization_confidence_decay_seconds,
)
robot = RobotController(hardware, settings.max_motor_percent, awareness=awareness)
navigator = SupervisedNavigator(
    robot=robot,
    odometry=odometry,
    occupancy_map=occupancy_map,
    planner=planner,
    awareness=awareness,
    lidar=lidar,
    hardware_mode=hardware.status().mode,
    localization=localizer,
    min_localization_confidence=settings.localization_nav_min_confidence,
    hardware_execution_enabled=settings.enable_supervised_navigation,
    allow_unknown=settings.navigation_allow_unknown,
    max_goal_distance_cm=settings.navigation_max_goal_distance_cm,
    max_linear=settings.navigation_max_linear,
    max_angular=settings.navigation_max_angular,
    waypoint_tolerance_cm=settings.navigation_waypoint_tolerance_cm,
    heading_tolerance_deg=settings.navigation_heading_tolerance_deg,
    timeout_seconds=settings.navigation_timeout_seconds,
)
dock = DockingFoundation(
    odometry,
    navigator,
    approach_distance_cm=settings.dock_approach_distance_cm,
)


def live_robot_context() -> str:
    spatial = awareness.context_text(camera.status())
    odom = odometry.status()
    map_state = occupancy_map.status()
    nav = navigator.status()
    localization = localizer.status()
    return (
        f"{spatial} "
        f"Local odometry pose: x={odom.pose.x_cm:.1f} cm, y={odom.pose.y_cm:.1f} cm, "
        f"heading={odom.pose.heading_deg:.1f} degrees; "
        f"odometry ready={'yes' if odom.ready and not odom.stale else 'no'}, "
        f"calibrated={'yes' if odom.calibrated else 'no'}. "
        f"Local occupancy map: ready={'yes' if map_state.ready else 'no'}, "
        f"{map_state.occupied_cells} occupied cells, {map_state.free_cells} free cells. "
        f"Localization: state={localization.state}, confidence={localization.confidence:.2f}, "
        f"ready={'yes' if localization.ready else 'no'}. "
        f"Navigation state: {nav.state}; {nav.reason or 'no active route'}."
    )


conversation = ConversationService(memory, llm, context_provider=live_robot_context)
speaker = LocalSpeaker(settings.enable_local_tts, settings.tts_command)
voice = OfflineVoiceAssistant(
    conversation,
    speaker,
    auto_start=settings.enable_voice_loop,
    engine=settings.voice_engine,
    wake_phrase=settings.wake_phrase,
    model_path=settings.vosk_model_path,
    microphone_device=settings.microphone_device,
    sample_rate=settings.voice_sample_rate,
    block_size=settings.voice_block_size,
    command_timeout_seconds=settings.voice_command_timeout_seconds,
)


@asynccontextmanager
async def lifespan(_: FastAPI):
    if settings.map_auto_load:
        occupancy_map.load()
        localizer.invalidate("Persistent map loaded; live relocalization is required")
    if settings.enable_odometry:
        odometry.start()
    if settings.enable_camera:
        camera.start()
    if settings.enable_lidar:
        lidar.start()
    if settings.enable_localization:
        localizer.start()
    if settings.enable_mapping:
        occupancy_map.start()
    if settings.enable_voice_loop:
        voice.start()
    yield
    if navigator.status().running:
        navigator.cancel("Application shutting down")
    occupancy_map.stop()
    if settings.map_auto_save and occupancy_map.status().dirty:
        occupancy_map.save()
    localizer.stop()
    odometry.stop()
    camera.stop()
    lidar.stop()
    voice.stop()
    hardware.close()


app = FastAPI(title="Ribitics Robot", version="0.5.0", lifespan=lifespan)
static_dir = Path(__file__).resolve().parent.parent / "static"
app.mount("/static", StaticFiles(directory=static_dir), name="static")


def require_control_token(
    x_ribitics_token: str | None = Header(default=None),
    token: str | None = Query(default=None),
) -> None:
    expected = settings.control_token
    if expected and expected != "change-me-before-network-use":
        if (x_ribitics_token or token) != expected:
            raise HTTPException(status_code=401, detail="Invalid Ribitics control token")


@app.get("/")
def dashboard() -> FileResponse:
    return FileResponse(static_dir / "index.html")


@app.get("/health")
def health() -> dict:
    return {
        "ok": True,
        "name": settings.name,
        "mode": settings.mode,
        "voice": voice.status().model_dump(),
        "camera": camera.status().model_dump(),
        "lidar": lidar.status().model_dump(),
        "spatial": awareness.status().model_dump(),
        "odometry": odometry.status().model_dump(),
        "localization": localizer.status().model_dump(),
        "map": occupancy_map.status().model_dump(),
        "navigation": navigator.status().model_dump(),
        "dock": dock.status().model_dump(),
    }


@app.get("/api/status", response_model=RobotStatus)
def status() -> RobotStatus:
    return hardware.status()


@app.get("/api/spatial/status", response_model=SpatialStatus)
def spatial_status() -> SpatialStatus:
    return awareness.status()


@app.post("/api/drive", dependencies=[Depends(require_control_token)])
def drive(command: DriveCommand) -> RobotStatus:
    if navigator.status().running:
        navigator.cancel("Manual drive took control")
    try:
        robot.drive(command.linear, command.angular)
    except UnsafeDriveError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return hardware.status()


@app.post("/api/stop", dependencies=[Depends(require_control_token)])
def stop() -> RobotStatus:
    if navigator.status().running:
        navigator.cancel("Operator STOP")
    robot.stop()
    return hardware.status()


@app.post(
    "/api/simulation/sensors",
    response_model=SpatialStatus,
    dependencies=[Depends(require_control_token)],
)
def simulation_sensors(sensors: SimulationSensors) -> SpatialStatus:
    if not isinstance(hardware, SimulatedHardware):
        raise HTTPException(status_code=409, detail="Sensor simulation is only available in simulation mode")
    hardware.set_sensors(sensors)
    return awareness.status()


@app.post(
    "/api/simulation/map-obstacle",
    response_model=MapStatus,
    dependencies=[Depends(require_control_token)],
)
def simulation_map_obstacle(obstacle: SimulationMapObstacle) -> MapStatus:
    if not isinstance(hardware, SimulatedHardware):
        raise HTTPException(status_code=409, detail="Map simulation is only available in simulation mode")
    return occupancy_map.add_virtual_obstacle(
        obstacle.x_cm,
        obstacle.y_cm,
        obstacle.radius_cm,
    )


@app.get("/api/odometry/status", response_model=OdometryStatus)
def odometry_status() -> OdometryStatus:
    return odometry.status()


@app.post(
    "/api/odometry/reset",
    response_model=OdometryStatus,
    dependencies=[Depends(require_control_token)],
)
def odometry_reset(pose: Pose2D) -> OdometryStatus:
    if navigator.status().running:
        navigator.cancel("Odometry reset by operator")
    robot.stop()
    result = odometry.reset(pose)
    localizer.invalidate("Odometry pose was reset by operator")
    return result


@app.get("/api/map/status", response_model=MapStatus)
def map_status() -> MapStatus:
    return occupancy_map.status()


@app.post(
    "/api/map/save",
    response_model=MapPersistenceResult,
    dependencies=[Depends(require_control_token)],
)
def map_save() -> MapPersistenceResult:
    robot.stop()
    return occupancy_map.save()


@app.post(
    "/api/map/load",
    response_model=MapPersistenceResult,
    dependencies=[Depends(require_control_token)],
)
def map_load() -> MapPersistenceResult:
    if navigator.status().running:
        navigator.cancel("Persistent map loaded by operator")
    robot.stop()
    result = occupancy_map.load()
    if result.success:
        localizer.invalidate("Persistent map loaded; relocalization is required")
    return result


@app.post(
    "/api/map/learning",
    response_model=MapStatus,
    dependencies=[Depends(require_control_token)],
)
def map_learning(request: MapLearningRequest) -> MapStatus:
    if navigator.status().running:
        navigator.cancel("Map learning mode changed")
    robot.stop()
    status = occupancy_map.set_learning(request.enabled)
    localizer.invalidate(
        "Map learning resumed" if request.enabled else "Reference map frozen; localization required"
    )
    return status


@app.get(
    "/api/map/snapshot",
    response_model=MapSnapshot,
    dependencies=[Depends(require_control_token)],
)
def map_snapshot() -> MapSnapshot:
    return occupancy_map.snapshot()


@app.post(
    "/api/map/clear",
    response_model=MapStatus,
    dependencies=[Depends(require_control_token)],
)
def map_clear() -> MapStatus:
    if navigator.status().running:
        navigator.cancel("Map cleared by operator")
    robot.stop()
    result = occupancy_map.clear()
    localizer.invalidate("Reference map was cleared")
    return result


@app.get("/api/localization/status", response_model=LocalizationStatus)
def localization_status() -> LocalizationStatus:
    return localizer.status()


@app.post(
    "/api/localization/match",
    response_model=ScanMatchResult,
    dependencies=[Depends(require_control_token)],
)
def localization_match(request: RelocalizeRequest) -> ScanMatchResult:
    if navigator.status().running:
        navigator.cancel("Localization match requested")
    robot.stop()
    return localizer.match_now(request)


@app.post(
    "/api/localization/relocalize",
    response_model=ScanMatchResult,
    dependencies=[Depends(require_control_token)],
)
def localization_relocalize(request: RelocalizeRequest) -> ScanMatchResult:
    if navigator.status().running:
        navigator.cancel("Explicit relocalization requested")
    robot.stop()
    return localizer.relocalize(request)


@app.get("/api/navigation/status", response_model=NavigationStatus)
def navigation_status() -> NavigationStatus:
    return navigator.status()


@app.get("/api/navigation/plan", response_model=NavigationPlan)
def navigation_current_plan() -> NavigationPlan:
    plan = navigator.current_plan()
    if plan is None:
        raise HTTPException(status_code=404, detail="No route has been planned")
    return plan


@app.post(
    "/api/navigation/plan",
    response_model=NavigationPlan,
    dependencies=[Depends(require_control_token)],
)
def navigation_plan(goal: NavigationGoal) -> NavigationPlan:
    try:
        return navigator.plan(goal)
    except NavigationError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post(
    "/api/navigation/start",
    response_model=NavigationStatus,
    dependencies=[Depends(require_control_token)],
)
def navigation_start(goal: NavigationGoal) -> NavigationStatus:
    try:
        return navigator.start(goal)
    except NavigationError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post(
    "/api/navigation/cancel",
    response_model=NavigationStatus,
    dependencies=[Depends(require_control_token)],
)
def navigation_cancel() -> NavigationStatus:
    return navigator.cancel("Cancelled by operator")


@app.post(
    "/api/navigation/replan",
    response_model=NavigationPlan,
    dependencies=[Depends(require_control_token)],
)
def navigation_replan() -> NavigationPlan:
    try:
        return navigator.replan_current_goal()
    except NavigationError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.get("/api/dock/status", response_model=DockStatus)
def dock_status() -> DockStatus:
    return dock.status()


@app.post(
    "/api/dock/set",
    response_model=DockStatus,
    dependencies=[Depends(require_control_token)],
)
def dock_set(pose: Pose2D) -> DockStatus:
    return dock.set_pose(pose)


@app.post(
    "/api/dock/set-current",
    response_model=DockStatus,
    dependencies=[Depends(require_control_token)],
)
def dock_set_current() -> DockStatus:
    try:
        return dock.set_current_pose()
    except NavigationError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post(
    "/api/dock/plan-return",
    response_model=NavigationPlan,
    dependencies=[Depends(require_control_token)],
)
def dock_plan_return() -> NavigationPlan:
    try:
        return dock.plan_return()
    except NavigationError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post(
    "/api/dock/start-return",
    response_model=NavigationStatus,
    dependencies=[Depends(require_control_token)],
)
def dock_start_return() -> NavigationStatus:
    try:
        return dock.start_return()
    except NavigationError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest) -> ChatResponse:
    response = await conversation.chat(request.message)
    speaker.speak(response.reply)
    return response


@app.get("/api/voice/status", response_model=VoiceStatus)
def voice_status() -> VoiceStatus:
    return voice.status()


@app.post(
    "/api/voice/start",
    response_model=VoiceStatus,
    dependencies=[Depends(require_control_token)],
)
def voice_start() -> VoiceStatus:
    return voice.start()


@app.post(
    "/api/voice/stop",
    response_model=VoiceStatus,
    dependencies=[Depends(require_control_token)],
)
def voice_stop() -> VoiceStatus:
    return voice.stop()


@app.get("/api/camera/status", response_model=CameraStatus)
def camera_status() -> CameraStatus:
    return camera.status()


@app.post(
    "/api/camera/start",
    response_model=CameraStatus,
    dependencies=[Depends(require_control_token)],
)
def camera_start() -> CameraStatus:
    return camera.start()


@app.post(
    "/api/camera/stop",
    response_model=CameraStatus,
    dependencies=[Depends(require_control_token)],
)
def camera_stop() -> CameraStatus:
    return camera.stop()


@app.get("/api/camera/snapshot", dependencies=[Depends(require_control_token)])
def camera_snapshot() -> Response:
    frame = camera.snapshot()
    if frame is None:
        raise HTTPException(status_code=503, detail="Camera has no frame available")
    return Response(
        content=frame,
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store, max-age=0"},
    )


@app.get("/api/lidar/status", response_model=LidarStatus)
def lidar_status() -> LidarStatus:
    return lidar.status()


@app.post(
    "/api/lidar/start",
    response_model=LidarStatus,
    dependencies=[Depends(require_control_token)],
)
def lidar_start() -> LidarStatus:
    return lidar.start()


@app.post(
    "/api/lidar/stop",
    response_model=LidarStatus,
    dependencies=[Depends(require_control_token)],
)
def lidar_stop() -> LidarStatus:
    return lidar.stop()


@app.get(
    "/api/lidar/scan",
    response_model=LidarScan,
    dependencies=[Depends(require_control_token)],
)
def lidar_scan() -> LidarScan:
    return lidar.scan()


@app.get("/api/memories", response_model=list[MemoryRecord])
def list_memories(limit: int = Query(default=50, ge=1, le=250)) -> list[MemoryRecord]:
    return memory.list(limit)


@app.post("/api/memories", response_model=MemoryRecord)
def create_memory(record: MemoryCreate) -> MemoryRecord:
    return memory.add(record.content, record.tags, record.importance)


@app.delete("/api/memories/{memory_id}")
def delete_memory(memory_id: int) -> dict:
    return {"deleted": memory.delete(memory_id)}


@app.post("/api/say")
def say(request: ChatRequest) -> dict:
    return {"spoken": speaker.speak(request.message), "text": request.message}
