from __future__ import annotations

import shlex
import shutil
import subprocess
import threading


class LocalSpeaker:
    def __init__(self, enabled: bool, command: str):
        self.enabled = enabled
        self.command = command
        self._process: subprocess.Popen | None = None
        self._lock = threading.Lock()

    @property
    def is_speaking(self) -> bool:
        with self._lock:
            if self._process is None:
                return False
            if self._process.poll() is None:
                return True
            self._process = None
            return False

    def speak(self, text: str) -> bool:
        parts = shlex.split(self.command)
        executable = parts[0] if parts else ""
        if not self.enabled or not executable or not shutil.which(executable):
            return False
        with self._lock:
            if self._process and self._process.poll() is None:
                self._process.terminate()
            self._process = subprocess.Popen(
                [*parts, text],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        return True
