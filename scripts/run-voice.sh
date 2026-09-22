#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export RIBITICS_ENABLE_VOICE_LOOP=true
export RIBITICS_ENABLE_LOCAL_TTS=true
exec uvicorn app.main:app --host 0.0.0.0 --port "${PORT:-8000}"
