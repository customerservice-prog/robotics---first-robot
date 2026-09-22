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
        live_context: str = "",
    ) -> tuple[str, str]:
        system = (
            f"You are {self.robot_name}, an upgradeable physical robot assistant. "
            "Be practical, concise, and truthful about what your hardware can actually do. Never claim "
            "a physical action completed unless telemetry confirms it. Never invent what the camera sees. "
            "If object recognition is not available, say so. Use supplied memories, recent conversation, "
            "and live robot context when relevant. If the user corrects you, accept the correction plainly.\n\n"
            "Relevant persistent memories:\n"
            + ("\n".join(f"- {memory}" for memory in memories) if memories else "- none")
            + "\n\nLive robot context:\n"
            + (live_context.strip() if live_context.strip() else "No live context supplied.")
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
        return self._fallback(message, memories, live_context), "fallback"

    def _fallback(self, message: str, memories: list[str], live_context: str = "") -> str:
        lowered = message.lower()
        if any(word in lowered for word in ("hello", "hey", "hi ", "hi!")) or lowered == "hi":
            return (
                f"Hey. I'm {self.robot_name}. My local AI model is offline, "
                "but my controls, memory, and safety services are running."
            )
        if "what do you remember" in lowered or "memory" in lowered:
            if not memories:
                return "I don't have a matching saved memory yet."
            return "I found these relevant memories: " + "; ".join(memories[:4])
        perception_question = any(
            phrase in lowered
            for phrase in (
                "what do you see",
                "can you see",
                "path clear",
                "is it clear",
                "obstacle",
                "distance",
                "camera",
                "bumper",
                "lidar",
                "sensor",
            )
        )
        if perception_question and live_context.strip():
            return "My live sensors report: " + live_context.strip()
        if "status" in lowered:
            if live_context.strip():
                return "My software is running. " + live_context.strip()
            return (
                "My software is running. Open the control panel for live motor, "
                "connection, and safety status."
            )
        return (
            "I heard you. My local language model is not connected yet, so I'm using my basic "
            "offline responder. You can still teach me memories, inspect live sensors, and control "
            "the robot manually."
        )
