from __future__ import annotations

import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

from app.models import MemoryRecord


class MemoryStore:
    def __init__(self, path: str):
        self.path = path
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._init_db()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path)
        connection.row_factory = sqlite3.Row
        return connection

    def _init_db(self) -> None:
        with self._connect() as db:
            db.execute(
                """
                CREATE TABLE IF NOT EXISTS memories (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    content TEXT NOT NULL,
                    tags TEXT NOT NULL DEFAULT '[]',
                    importance INTEGER NOT NULL DEFAULT 5,
                    created_at TEXT NOT NULL
                )
                """
            )
            db.execute("CREATE INDEX IF NOT EXISTS idx_memories_created ON memories(created_at)")
            db.execute(
                """
                CREATE TABLE IF NOT EXISTS conversation_messages (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    role TEXT NOT NULL CHECK(role IN ('user', 'assistant')),
                    content TEXT NOT NULL,
                    created_at TEXT NOT NULL
                )
                """
            )
            db.execute(
                "CREATE INDEX IF NOT EXISTS idx_conversation_created "
                "ON conversation_messages(created_at)"
            )

    def add(self, content: str, tags: list[str] | None = None, importance: int = 5) -> MemoryRecord:
        created_at = datetime.now(timezone.utc)
        clean_tags = [tag.strip().lower() for tag in (tags or []) if tag.strip()]
        clean_content = content.strip()
        with self._connect() as db:
            cursor = db.execute(
                "INSERT INTO memories(content, tags, importance, created_at) VALUES(?, ?, ?, ?)",
                (clean_content, json.dumps(clean_tags), importance, created_at.isoformat()),
            )
            memory_id = int(cursor.lastrowid)
        return MemoryRecord(
            id=memory_id,
            content=clean_content,
            tags=clean_tags,
            importance=importance,
            created_at=created_at,
        )

    def list(self, limit: int = 100) -> list[MemoryRecord]:
        with self._connect() as db:
            rows = db.execute(
                "SELECT * FROM memories ORDER BY importance DESC, id DESC LIMIT ?", (limit,)
            ).fetchall()
        return [self._row_to_record(row) for row in rows]

    def search(self, query: str, limit: int = 8) -> list[MemoryRecord]:
        words = [word.lower() for word in query.split() if len(word) >= 3]
        if not words:
            return self.list(limit)
        scored: list[tuple[int, MemoryRecord]] = []
        for record in self.list(250):
            haystack = f"{record.content} {' '.join(record.tags)}".lower()
            score = sum(1 for word in words if word in haystack) + record.importance // 4
            if score:
                scored.append((score, record))
        scored.sort(key=lambda item: (item[0], item[1].importance, item[1].id), reverse=True)
        return [record for _, record in scored[:limit]]

    def delete(self, memory_id: int) -> bool:
        with self._connect() as db:
            cursor = db.execute("DELETE FROM memories WHERE id = ?", (memory_id,))
            return cursor.rowcount > 0

    def add_conversation_message(self, role: str, content: str) -> None:
        if role not in {"user", "assistant"}:
            raise ValueError("conversation role must be 'user' or 'assistant'")
        clean_content = content.strip()
        if not clean_content:
            return
        created_at = datetime.now(timezone.utc).isoformat()
        with self._connect() as db:
            db.execute(
                "INSERT INTO conversation_messages(role, content, created_at) VALUES(?, ?, ?)",
                (role, clean_content, created_at),
            )

    def recent_conversation(self, limit: int = 12) -> list[dict[str, str]]:
        with self._connect() as db:
            rows = db.execute(
                "SELECT role, content FROM conversation_messages ORDER BY id DESC LIMIT ?",
                (limit,),
            ).fetchall()
        return [
            {"role": row["role"], "content": row["content"]}
            for row in reversed(rows)
        ]

    @staticmethod
    def _row_to_record(row: sqlite3.Row) -> MemoryRecord:
        return MemoryRecord(
            id=row["id"],
            content=row["content"],
            tags=json.loads(row["tags"]),
            importance=row["importance"],
            created_at=datetime.fromisoformat(row["created_at"]),
        )
