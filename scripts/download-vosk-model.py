#!/usr/bin/env python3
"""Download and safely unpack Ribitics' small English Vosk model."""

from __future__ import annotations

import argparse
import tempfile
import urllib.request
import zipfile
from pathlib import Path

DEFAULT_URL = "https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip"
DEFAULT_DEST = Path("models")


def safe_extract(archive: zipfile.ZipFile, destination: Path) -> None:
    root = destination.resolve()
    for member in archive.infolist():
        target = (destination / member.filename).resolve()
        if root != target and root not in target.parents:
            raise RuntimeError(f"Refusing unsafe archive member: {member.filename}")
    archive.extractall(destination)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--destination", type=Path, default=DEFAULT_DEST)
    args = parser.parse_args()

    args.destination.mkdir(parents=True, exist_ok=True)
    model_name = Path(args.url).name.removesuffix(".zip")
    final_dir = args.destination / model_name
    if final_dir.exists():
        print(f"Model already exists: {final_dir}")
        return 0

    with tempfile.TemporaryDirectory(prefix="ribitics-vosk-") as temp_dir:
        archive_path = Path(temp_dir) / "model.zip"
        print(f"Downloading {args.url}")
        urllib.request.urlretrieve(args.url, archive_path)
        print("Extracting model...")
        with zipfile.ZipFile(archive_path) as archive:
            safe_extract(archive, args.destination)

    if not final_dir.exists():
        raise RuntimeError(f"Download finished but expected model folder was not found: {final_dir}")
    print(f"Ready: {final_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
