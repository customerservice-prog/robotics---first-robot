from __future__ import annotations

import httpx


class LocalLLM:
    """Small Ollama client. Falls back cleanly when Ollama is not installed."""

    def __init__(self, base_url: str, model: str, robot_name: str):
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.robot_name = robot_name

    async def reply(
        self,
        message: str,
        memories: list[str],
        history: list[dict[str, str]] | None = None,
    ) -> tuple[str, str]:
        system = (
            f"You are {self.robot_name}, an upgradeable physical robot assistant. "
            "Be practical, concise, truthful about what your hardware can actually do, and never claim "
            "a physical action completed unless telemetry confirms it. Use supplied memories and recent "
            "conversation context when relevant. If the user corrects you, accept the correction plainly.\n\n"
            "Relevant persistent memories:\n"
            + ("\n".join(f"- {memory}" for memory in memories) if memories else "- none")
        )
        messages = [{"role": "system", "content": system}]
        for item in (history or [])[-12:]:
            role = item.get("role")
            content = item.get("content", "").strip()
            if role in {"user", "assistant"} and content:
                messages.append({"role": role, "content": content})
        messages.append({"role": "user", "content": message})
        payload = {
            "model": self.model,
            "stream": False,
            "messages": messages,
        }
        try:
            async with httpx.AsyncClient(timeout=45) as client:
                response = await client.post(f"{self.base_url}/api/chat", json=payload)
                response.raise_for_status()
            content = response.json().get("message", {}).get("content", "").strip()
            if content:
                return content, f"ollama:{self.model}"
        except (httpx.HTTPError, ValueError):
            pass
        return self._fallback(message, memories), "fallback"

    def _fallback(self, message: str, memories: list[str]) -> str:
        lowered = message.lower()
        if any(word in lowered for word in ("hello", "hey", "hi ", "hi!")) or lowered == "hi":
            return (
                f"Hey. I'm {self.robot_name}. My local AI model is offline, "
                "but my controls and memory are running."
            )
        if "what do you remember" in lowered or "memory" in lowered:
            if not memories:
                return "I don't have a matching saved memory yet."
            return "I found these relevant memories: " + "; ".join(memories[:4])
        if "status" in lowered:
            return (
                "My software is running. Open the control panel for live motor, "
                "connection, and safety status."
            )
        return (
            "I heard you. My local language model is not connected yet, so I'm using my basic "
            "offline responder. You can still teach me memories and control the robot."
        )
