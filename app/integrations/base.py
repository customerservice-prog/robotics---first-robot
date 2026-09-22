from typing import Protocol


class RobotIntegration(Protocol):
    name: str

    async def query(self, action: str, payload: dict) -> dict: ...
