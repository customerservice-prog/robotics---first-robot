from __future__ import annotations

import asyncio
import json
import queue
import re
import threading
import time
from pathlib import Path
from typing import Any

from app.models import VoiceStatus
from app.services.conversation import ConversationService
from app.services.speech import LocalSpeaker


def _normalize_phrase(text: str) -> str:
    return " ".join(re.findall(r"[a-z0-9]+", text.lower()))


def extract_wake_command(transcript: str, wake_phrase: str) -> tuple[bool, str]:
    """Return whether the wake phrase was heard and any command following it."""
    normalized = _normalize_phrase(transcript)
    wake = _normalize_phrase(wake_phrase)
    if not normalized or not wake:
        return False, ""
    if normalized == wake:
        return True, ""
    prefix = wake + " "
    if normalized.startswith(prefix):
        return True, normalized[len(prefix) :].strip()
    return False, ""


class OfflineVoiceAssistant:
    """Always-listening, fully local Vosk microphone loop.

    Heavy audio dependencies are imported only inside the worker thread so the core
    robot, tests, and simulation continue to run without voice packages installed.
    """

    def __init__(
        self,
        conversation: ConversationService,
        speaker: LocalSpeaker,
        *,
        auto_start: bool,
        engine: str,
        wake_phrase: str,
        model_path: str,
        microphone_device: str | None,
        sample_rate: int,
        block_size: int,
        command_timeout_seconds: float,
    ) -> None:
        self.conversation = conversation
        self.speaker = speaker
        self.auto_start = auto_start
        self.engine = engine.lower().strip()
        self.wake_phrase = wake_phrase.strip()
        self.model_path = model_path
        self.microphone_device = microphone_device
        self.sample_rate = sample_rate
        self.block_size = block_size
        self.command_timeout_seconds = command_timeout_seconds

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._running = False
        self._ready = False
        self._state = "stopped"
        self._last_heard = ""
        self._last_reply = ""
        self._last_error = ""

    def start(self) -> VoiceStatus:
        if self._thread and self._thread.is_alive():
            return self.status()
        self._stop_event.clear()
        self._set_status(running=True, ready=False, state="starting", last_error="")
        self._thread = threading.Thread(target=self._run, name="ribitics-voice", daemon=True)
        self._thread.start()
        return self.status()

    def stop(self) -> VoiceStatus:
        self._stop_event.set()
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.5)
        self._set_status(running=False, ready=False, state="stopped")
        return self.status()

    def status(self) -> VoiceStatus:
        with self._lock:
            return VoiceStatus(
                auto_start=self.auto_start,
                running=self._running,
                ready=self._ready,
                state=self._state,
                engine=self.engine,
                wake_phrase=self.wake_phrase,
                model_path=self.model_path,
                last_heard=self._last_heard,
                last_reply=self._last_reply,
                last_error=self._last_error,
            )

    def _set_status(
        self,
        *,
        running: bool | None = None,
        ready: bool | None = None,
        state: str | None = None,
        last_heard: str | None = None,
        last_reply: str | None = None,
        last_error: str | None = None,
    ) -> None:
        with self._lock:
            if running is not None:
                self._running = running
            if ready is not None:
                self._ready = ready
            if state is not None:
                self._state = state
            if last_heard is not None:
                self._last_heard = last_heard
            if last_reply is not None:
                self._last_reply = last_reply
            if last_error is not None:
                self._last_error = last_error

    def _run(self) -> None:
        try:
            if self.engine != "vosk":
                raise RuntimeError(f"Unsupported offline voice engine: {self.engine}")
            model_dir = Path(self.model_path)
            if not model_dir.exists():
                raise RuntimeError(
                    f"Vosk model not found at {model_dir}. Run scripts/download-vosk-model.py first."
                )

            try:
                import sounddevice as sd
                from vosk import KaldiRecognizer, Model, SetLogLevel
            except ImportError as exc:
                raise RuntimeError(
                    "Offline voice packages are missing. Install with: pip install -e '.[voice]'"
                ) from exc

            SetLogLevel(-1)
            model = Model(str(model_dir))
            recognizer = KaldiRecognizer(model, self.sample_rate)
            audio: queue.Queue[bytes] = queue.Queue(maxsize=64)

            def callback(indata: Any, _frames: int, _time_info: Any, _status: Any) -> None:
                if self._stop_event.is_set():
                    return
                try:
                    audio.put_nowait(bytes(indata))
                except queue.Full:
                    try:
                        audio.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        audio.put_nowait(bytes(indata))
                    except queue.Full:
                        pass

            device: int | str | None = self.microphone_device
            if isinstance(device, str) and device.strip().isdigit():
                device = int(device.strip())

            with sd.RawInputStream(
                samplerate=self.sample_rate,
                blocksize=self.block_size,
                device=device,
                dtype="int16",
                channels=1,
                callback=callback,
            ):
                self._set_status(running=True, ready=True, state="waiting_for_wake", last_error="")
                waiting_for_command = False
                command_deadline = 0.0

                while not self._stop_event.is_set():
                    if self.speaker.is_speaking:
                        self._drain_audio(audio)
                        recognizer.Reset()
                        time.sleep(0.05)
                        continue

                    if waiting_for_command and time.monotonic() >= command_deadline:
                        waiting_for_command = False
                        recognizer.Reset()
                        self._set_status(state="waiting_for_wake")

                    try:
                        chunk = audio.get(timeout=0.25)
                    except queue.Empty:
                        continue

                    if not recognizer.AcceptWaveform(chunk):
                        continue
                    result = json.loads(recognizer.Result())
                    transcript = result.get("text", "").strip()
                    if not transcript:
                        continue
                    self._set_status(last_heard=transcript)

                    if waiting_for_command:
                        waiting_for_command = False
                        self._handle_command(transcript)
                        recognizer.Reset()
                        continue

                    woke, command = extract_wake_command(transcript, self.wake_phrase)
                    if not woke:
                        continue
                    if command:
                        self._handle_command(command)
                        recognizer.Reset()
                    else:
                        waiting_for_command = True
                        command_deadline = time.monotonic() + self.command_timeout_seconds
                        self._set_status(state="listening_for_command")

        except Exception as exc:
            self._set_status(
                running=False,
                ready=False,
                state="error",
                last_error=str(exc),
            )
            return
        finally:
            if self._stop_event.is_set():
                self._set_status(running=False, ready=False, state="stopped")

    def _handle_command(self, command: str) -> None:
        clean = command.strip()
        if not clean:
            self._set_status(state="waiting_for_wake")
            return
        self._set_status(state="thinking", last_heard=clean)
        try:
            response = asyncio.run(self.conversation.chat(clean))
            self._set_status(state="speaking", last_reply=response.reply, last_error="")
            self.speaker.speak(response.reply)
        except Exception as exc:
            self._set_status(last_error=f"Voice command failed: {exc}")
        finally:
            self._set_status(state="waiting_for_wake")

    @staticmethod
    def _drain_audio(audio: queue.Queue[bytes]) -> None:
        while True:
            try:
                audio.get_nowait()
            except queue.Empty:
                return
