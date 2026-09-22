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
