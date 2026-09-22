from __future__ import annotations

import threading
import time
from datetime import datetime, timezone

from app.models import CameraStatus


class CameraService:
    """Optional OpenCV USB-camera capture.

    OpenCV is imported only inside the worker thread. That keeps the rest of Ribitics
    usable on machines without camera dependencies or a camera attached.
    """

    def __init__(
        self,
        *,
        auto_start: bool,
        device: str,
        width: int,
        height: int,
        fps: float,
        jpeg_quality: int,
        motion_threshold: float,
    ) -> None:
        self.auto_start = auto_start
        self.device = str(device)
        self.requested_width = max(160, int(width))
        self.requested_height = max(120, int(height))
        self.requested_fps = max(1.0, float(fps))
        self.jpeg_quality = max(30, min(95, int(jpeg_quality)))
        self.motion_threshold = max(0.1, float(motion_threshold))

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._latest_jpeg: bytes | None = None
        self._running = False
        self._ready = False
        self._state = "stopped"
        self._width = 0
        self._height = 0
        self._fps = 0.0
        self._frames_captured = 0
        self._motion_detected = False
        self._motion_score = 0.0
        self._last_frame_at: datetime | None = None
        self._last_error = ""

    def start(self) -> CameraStatus:
        if self._thread and self._thread.is_alive():
            return self.status()
        self._stop_event.clear()
        with self._lock:
            self._running = True
            self._ready = False
            self._state = "starting"
            self._last_error = ""
        self._thread = threading.Thread(target=self._run, name="ribitics-camera", daemon=True)
        self._thread.start()
        return self.status()

    def stop(self) -> CameraStatus:
        self._stop_event.set()
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=3.0)
        with self._lock:
            self._running = False
            self._ready = False
            self._state = "stopped"
            self._latest_jpeg = None
        return self.status()

    def status(self) -> CameraStatus:
        with self._lock:
            return CameraStatus(
                auto_start=self.auto_start,
                running=self._running,
                ready=self._ready,
                state=self._state,
                device=self.device,
                width=self._width,
                height=self._height,
                fps=self._fps,
                frames_captured=self._frames_captured,
                motion_detected=self._motion_detected,
                motion_score=round(self._motion_score, 2),
                last_frame_at=self._last_frame_at,
                last_error=self._last_error,
            )

    def snapshot(self) -> bytes | None:
        with self._lock:
            return bytes(self._latest_jpeg) if self._latest_jpeg is not None else None

    def _run(self) -> None:
        capture = None
        try:
            try:
                import cv2
            except ImportError as exc:
                raise RuntimeError(
                    "Camera support is not installed. Install with: pip install -e '.[vision]'"
                ) from exc

            device: int | str = self.device
            if self.device.strip().lstrip("-").isdigit():
                device = int(self.device.strip())

            capture = cv2.VideoCapture(device)
            if not capture.isOpened():
                raise RuntimeError(f"Could not open camera device {self.device}")

            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.requested_width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.requested_height)
            capture.set(cv2.CAP_PROP_FPS, self.requested_fps)

            previous_gray = None
            started = time.monotonic()
            consecutive_failures = 0

            while not self._stop_event.is_set():
                ok, frame = capture.read()
                if not ok or frame is None:
                    consecutive_failures += 1
                    if consecutive_failures >= 20:
                        raise RuntimeError("Camera stopped returning frames")
                    time.sleep(0.05)
                    continue

                consecutive_failures = 0
                height, width = frame.shape[:2]
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                gray = cv2.GaussianBlur(gray, (21, 21), 0)

                motion_score = 0.0
                motion_detected = False
                if previous_gray is not None:
                    difference = cv2.absdiff(previous_gray, gray)
                    motion_score = float(difference.mean())
                    motion_detected = motion_score >= self.motion_threshold
                previous_gray = gray

                encode_ok, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
                )
                if not encode_ok:
                    continue

                now = datetime.now(timezone.utc)
                elapsed = max(0.001, time.monotonic() - started)
                with self._lock:
                    self._latest_jpeg = encoded.tobytes()
                    self._running = True
                    self._ready = True
                    self._state = "streaming"
                    self._width = int(width)
                    self._height = int(height)
                    self._frames_captured += 1
                    self._fps = self._frames_captured / elapsed
                    self._motion_score = motion_score
                    self._motion_detected = motion_detected
                    self._last_frame_at = now
                    self._last_error = ""

                target_delay = 1.0 / self.requested_fps
                time.sleep(min(target_delay, 0.1))

        except Exception as exc:
            with self._lock:
                self._running = False
                self._ready = False
                self._state = "error"
                self._last_error = str(exc)
                self._latest_jpeg = None
        finally:
            if capture is not None:
                try:
                    capture.release()
                except Exception:
                    pass
            if self._stop_event.is_set():
                with self._lock:
                    self._running = False
                    self._ready = False
                    self._state = "stopped"
