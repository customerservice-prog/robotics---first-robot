import shutil
import subprocess


class LocalSpeaker:
    def __init__(self, enabled: bool, command: str):
        self.enabled = enabled
        self.command = command

    def speak(self, text: str) -> bool:
        if not self.enabled or not shutil.which(self.command):
            return False
        subprocess.Popen(
            [self.command, text],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return True
