"""MediaPipe HandLandmarker wrapper that yields :class:`HandPose` objects per frame.

Uses the MediaPipe Tasks API; the legacy ``mp.solutions.hands`` module was
removed in mediapipe 1.x.
"""

from __future__ import annotations

import os
import urllib.request
from pathlib import Path

import numpy as np

from hand_pose import HandPose, build_pose

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/1/hand_landmarker.task"
)
MODEL_PATH = Path(__file__).parent / "models" / "hand_landmarker.task"


def ensure_model(path: Path = MODEL_PATH) -> Path:
    """Download the hand_landmarker bundle on first use."""
    if path.exists() and path.stat().st_size > 0:
        return path
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".part")
    print(f"downloading hand landmarker model -> {path}")
    with urllib.request.urlopen(MODEL_URL) as response, tmp.open("wb") as fh:  # noqa: S310 - fixed https URL
        fh.write(response.read())
    os.replace(tmp, path)
    return path


class HandTracker:
    def __init__(
        self,
        max_hands: int = 2,
        detection_confidence: float = 0.6,
        presence_confidence: float = 0.5,
        tracking_confidence: float = 0.5,
        smoothing: float = 0.5,
        model_path: Path | None = None,
    ) -> None:
        import mediapipe as mp  # imported lazily so the module can be inspected without it
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision

        self._mp = mp
        options = vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(ensure_model(model_path or MODEL_PATH))),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=max_hands,
            min_hand_detection_confidence=detection_confidence,
            min_hand_presence_confidence=presence_confidence,
            min_tracking_confidence=tracking_confidence,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)
        self._alpha = float(np.clip(smoothing, 0.0, 0.95))
        self._filtered: dict[str, np.ndarray] = {}

    def __enter__(self) -> "HandTracker":
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def close(self) -> None:
        self._landmarker.close()

    def process(self, frame_bgr: np.ndarray, timestamp_ms: int) -> list[HandPose]:
        """``frame_bgr`` is an OpenCV BGR image; timestamps must increase monotonically."""
        height, width = frame_bgr.shape[:2]
        rgb = np.ascontiguousarray(frame_bgr[:, :, ::-1])
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(image, timestamp_ms)

        if not result.hand_landmarks or not result.hand_world_landmarks:
            self._filtered.clear()
            return []

        poses: list[HandPose] = []
        for image_lms, world_lms, handed in zip(
            result.hand_landmarks,
            result.hand_world_landmarks,
            result.handedness,
        ):
            label = handed[0].category_name
            score = float(handed[0].score)
            world = np.array([[p.x, p.y, p.z] for p in world_lms], dtype=np.float64)
            pixels = np.array([[p.x * width, p.y * height] for p in image_lms], dtype=np.float64)
            world = self._smooth(label, world)
            poses.append(build_pose(label, score, world, pixels))
        return poses

    def _smooth(self, key: str, world: np.ndarray) -> np.ndarray:
        if self._alpha <= 0.0:
            return world
        previous = self._filtered.get(key)
        smoothed = world if previous is None else self._alpha * previous + (1.0 - self._alpha) * world
        self._filtered[key] = smoothed
        return smoothed
