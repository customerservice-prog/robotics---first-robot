$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")
$env:RIBITICS_ENABLE_VOICE_LOOP = "true"
$env:RIBITICS_ENABLE_LOCAL_TTS = "true"
uvicorn app.main:app --host 0.0.0.0 --port $(if ($env:PORT) { $env:PORT } else { "8000" })
