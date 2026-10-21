"""USB camera helpers."""

from __future__ import annotations

import glob
import time
from pathlib import Path

import cv2
import numpy as np


def device_name(index: int) -> str:
    """UVC product name exposed by the kernel, e.g. 'HD Webcam: HD Webcam'."""
    try:
        return Path(f"/sys/class/video4linux/video{index}/name").read_text().strip()
    except OSError:
        return "unknown"


def list_devices() -> list[dict]:
    """Probe every /dev/video* node and report the ones that really deliver frames.

    UVC cameras expose several nodes (capture + metadata); only some can capture.
    """
    devices: list[dict] = []
    for path in sorted(glob.glob("/dev/video*"), key=lambda p: int(p.replace("/dev/video", "") or 0)):
        try:
            index = int(path.replace("/dev/video", ""))
        except ValueError:
            continue
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        ok = cap.isOpened() and cap.read()[0]
        if ok:
            devices.append(
                {
                    "index": index,
                    "name": device_name(index),
                    "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                    "fps": round(cap.get(cv2.CAP_PROP_FPS), 1),
                }
            )
        cap.release()
    return devices


def available_devices() -> list[int]:
    return [d["index"] for d in list_devices()]


def open_camera(device: int | str, width: int, height: int, fps: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2 if isinstance(device, int) else cv2.CAP_ANY)
    if not cap.isOpened():
        raise RuntimeError(
            f"cannot open camera {device!r}; plug in the USB camera and check `ls /dev/video*` "
            f"(available now: {available_devices() or 'none'})"
        )
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    # BUFFERSIZE=1 is deliberately not set: on UVC cameras it halves the delivered
    # rate, and it is unnecessary because the consumer never falls behind.
    return cap


class StreamReader:
    """Synchronous reader: every decoded frame is consumed immediately.

    Inference (~9 ms) is far faster than the camera period (~36 ms), so frames are
    never queued behind the consumer and the pose always reflects the newest frame.
    ``max_fps`` lowers the input rate by draining the driver queue with cheap
    ``grab()`` calls, which keeps the next decoded frame current.
    """

    def __init__(self, cap: cv2.VideoCapture, max_fps: float | None = None) -> None:
        self._cap = cap
        native = cap.get(cv2.CAP_PROP_FPS) or 30.0
        # Drop whole frames rather than waiting on a clock: a time-based wait always
        # overshoots by one camera period and halves the achieved rate.
        self._skip = max(0, round(native / max_fps) - 1) if max_fps else 0

    def read(self) -> tuple[np.ndarray, float] | None:
        for _ in range(self._skip):
            if not self._cap.grab():
                return None
        ok, frame = self._cap.read()
        if not ok:
            return None
        return frame, time.monotonic()

    def release(self) -> None:
        self._cap.release()
