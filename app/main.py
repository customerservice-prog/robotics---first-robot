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
    DriveCommand,
    MemoryCreate,
    MemoryRecord,
    RobotStatus,
    SimulationSensors,
    SpatialStatus,
    VoiceStatus,
)
from app.perception.camera import CameraService
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
    hardware = SimulatedHardware(settings.name)

camera = CameraService(
    auto_start=settings.enable_camera,
    device=settings.camera_device,
    width=settings.camera_width,
    height=settings.camera_height,
    fps=settings.camera_fps,
    jpeg_quality=settings.camera_jpeg_quality,
    motion_threshold=settings.camera_motion_threshold,
)
awareness = SpatialAwareness(
    hardware,
    stop_cm=settings.obstacle_stop_cm,
    warn_cm=settings.obstacle_warn_cm,
    require_proximity_for_forward=settings.require_proximity_for_forward,
)


def live_robot_context() -> str:
    return awareness.context_text(camera.status())


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
robot = RobotController(hardware, settings.max_motor_percent, awareness=awareness)


@asynccontextmanager
async def lifespan(_: FastAPI):
    if settings.enable_camera:
        camera.start()
    if settings.enable_voice_loop:
        voice.start()
    yield
    camera.stop()
    voice.stop()
    hardware.close()


app = FastAPI(title="Ribitics Robot", version="0.3.0", lifespan=lifespan)
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
        "spatial": awareness.status().model_dump(),
    }


@app.get("/api/status", response_model=RobotStatus)
def status() -> RobotStatus:
    return hardware.status()


@app.get("/api/spatial/status", response_model=SpatialStatus)
def spatial_status() -> SpatialStatus:
    return awareness.status()


@app.post("/api/drive", dependencies=[Depends(require_control_token)])
def drive(command: DriveCommand) -> RobotStatus:
    try:
        robot.drive(command.linear, command.angular)
    except UnsafeDriveError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return hardware.status()


@app.post("/api/stop", dependencies=[Depends(require_control_token)])
def stop() -> RobotStatus:
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
