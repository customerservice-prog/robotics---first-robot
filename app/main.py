from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import Depends, FastAPI, Header, HTTPException, Query
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from app.config import get_settings
from app.hardware import SimulatedHardware
from app.models import ChatRequest, ChatResponse, DriveCommand, MemoryCreate, MemoryRecord, RobotStatus
from app.robot import RobotController
from app.services.conversation import ConversationService
from app.services.llm import LocalLLM
from app.services.memory import MemoryStore
from app.services.speech import LocalSpeaker

settings = get_settings()
memory = MemoryStore(settings.database_path)
llm = LocalLLM(settings.ollama_url, settings.ollama_model, settings.name)
conversation = ConversationService(memory, llm)
speaker = LocalSpeaker(settings.enable_local_tts, settings.tts_command)

if settings.mode.lower() == "esp32":
    from app.hardware.esp32_serial import ESP32SerialHardware

    hardware = ESP32SerialHardware(settings.name, settings.serial_port, settings.serial_baud)
else:
    hardware = SimulatedHardware(settings.name)
robot = RobotController(hardware, settings.max_motor_percent)


@asynccontextmanager
async def lifespan(_: FastAPI):
    yield
    hardware.close()


app = FastAPI(title="Ribitics Robot", version="0.1.0", lifespan=lifespan)
static_dir = Path(__file__).resolve().parent.parent / "static"
app.mount("/static", StaticFiles(directory=static_dir), name="static")


def require_control_token(
    x_ribitics_token: str | None = Header(default=None),
    token: str | None = Query(default=None),
) -> None:
    # Localhost/dev convenience while still making accidental LAN exposure visible.
    expected = settings.control_token
    if expected and expected != "change-me-before-network-use":
        if (x_ribitics_token or token) != expected:
            raise HTTPException(status_code=401, detail="Invalid Ribitics control token")


@app.get("/")
def dashboard() -> FileResponse:
    return FileResponse(static_dir / "index.html")


@app.get("/health")
def health() -> dict:
    return {"ok": True, "name": settings.name, "mode": settings.mode}


@app.get("/api/status", response_model=RobotStatus)
def status() -> RobotStatus:
    return hardware.status()


@app.post("/api/drive", dependencies=[Depends(require_control_token)])
def drive(command: DriveCommand) -> RobotStatus:
    robot.drive(command.linear, command.angular)
    return hardware.status()


@app.post("/api/stop", dependencies=[Depends(require_control_token)])
def stop() -> RobotStatus:
    robot.stop()
    return hardware.status()


@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest) -> ChatResponse:
    response = await conversation.chat(request.message)
    speaker.speak(response.reply)
    return response


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
