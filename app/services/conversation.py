import re

from app.models import ChatResponse
from app.services.llm import LocalLLM
from app.services.memory import MemoryStore


REMEMBER_PATTERNS = [
    re.compile(r"^remember(?: that)?\s+(.+)$", re.IGNORECASE),
    re.compile(r"^learn(?: that)?\s+(.+)$", re.IGNORECASE),
]


class ConversationService:
    def __init__(self, memory: MemoryStore, llm: LocalLLM):
        self.memory = memory
        self.llm = llm

    async def chat(self, message: str) -> ChatResponse:
        cleaned = message.strip()
        history = self.memory.recent_conversation(limit=12)
        self.memory.add_conversation_message("user", cleaned)

        for pattern in REMEMBER_PATTERNS:
            match = pattern.match(cleaned)
            if match:
                fact = match.group(1).strip().rstrip(".")
                self.memory.add(fact, tags=["learned", "conversation"], importance=7)
                reply = f"Okay. I'll remember: {fact}."
                self.memory.add_conversation_message("assistant", reply)
                return ChatResponse(
                    reply=reply,
                    remembered=True,
                    model="memory",
                )

        memories = [record.content for record in self.memory.search(cleaned, limit=8)]
        reply, model = await self.llm.reply(cleaned, memories, history)
        self.memory.add_conversation_message("assistant", reply)
        return ChatResponse(reply=reply, model=model)
