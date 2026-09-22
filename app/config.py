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
    max_motor_percent: int = 65
    control_token: str = "change-me-before-network-use"

    def ensure_data_dir(self) -> None:
        Path(self.database_path).parent.mkdir(parents=True, exist_ok=True)


@lru_cache
def get_settings() -> Settings:
    settings = Settings()
    settings.ensure_data_dir()
    return settings
