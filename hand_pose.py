"""Geometric hand-pose features derived from 21 MediaPipe hand landmarks.

Landmark index convention (MediaPipe Hands):
    0  wrist
    1-4   thumb   : CMC, MCP, IP, TIP
    5-8   index   : MCP, PIP, DIP, TIP
    9-12  middle  : MCP, PIP, DIP, TIP
    13-16 ring    : MCP, PIP, DIP, TIP
    17-20 pinky   : MCP, PIP, DIP, TIP
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

WRIST = 0
INDEX_MCP = 5
MIDDLE_MCP = 9
RING_MCP = 13
PINKY_MCP = 17

FINGER_CHAINS: dict[str, tuple[int, int, int, int, int]] = {
    # (root, joint1, joint2, joint3, tip) -> angles are measured at joint1..joint3
    "thumb": (WRIST, 1, 2, 3, 4),
    "index": (WRIST, 5, 6, 7, 8),
    "middle": (WRIST, 9, 10, 11, 12),
    "ring": (WRIST, 13, 14, 15, 16),
    "pinky": (WRIST, 17, 18, 19, 20),
}

FINGERS = tuple(FINGER_CHAINS)

# Empirical flexion range (sum of the three joint angles, degrees) used to
# normalise curl into 0..1. Tune per operator if the mapping feels off.
CURL_RANGE_DEG: dict[str, tuple[float, float]] = {
    "thumb": (20.0, 130.0),
    "index": (15.0, 250.0),
    "middle": (15.0, 250.0),
    "ring": (15.0, 250.0),
    "pinky": (15.0, 250.0),
}


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else np.zeros_like(v)


def _angle_deg(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """Interior angle at ``b`` turned into a flexion angle (0 = straight)."""
    u, v = _unit(a - b), _unit(c - b)
    cos = float(np.clip(np.dot(u, v), -1.0, 1.0))
    return 180.0 - float(np.degrees(np.arccos(cos)))


@dataclass
class HandPose:
    """Everything one frame tells us about a single hand."""

    handedness: str  # "Left" / "Right"
    score: float
    landmarks: np.ndarray  # (21, 3) in metric-ish world space, origin at the hand centre
    pixels: np.ndarray  # (21, 2) image coordinates, for drawing
    joint_angles: dict[str, list[float]] = field(default_factory=dict)
    curls: dict[str, float] = field(default_factory=dict)
    spreads: dict[str, float] = field(default_factory=dict)
    palm_rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    palm_euler: tuple[float, float, float] = (0.0, 0.0, 0.0)
    palm_normal: np.ndarray = field(default_factory=lambda: np.zeros(3))
    palm_center: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def as_dict(self) -> dict:
        return {
            "handedness": self.handedness,
            "score": round(self.score, 4),
            "joint_angles": {k: [round(a, 2) for a in v] for k, v in self.joint_angles.items()},
            "curls": {k: round(v, 4) for k, v in self.curls.items()},
            "spreads": {k: round(v, 2) for k, v in self.spreads.items()},
            "palm_euler": tuple(round(a, 2) for a in self.palm_euler),
            "palm_normal": [round(float(x), 4) for x in self.palm_normal],
            "palm_center": [round(float(x), 4) for x in self.palm_center],
        }


def joint_angles(landmarks: np.ndarray) -> dict[str, list[float]]:
    """Flexion angle (deg) at the three movable joints of every finger."""
    out: dict[str, list[float]] = {}
    for name, chain in FINGER_CHAINS.items():
        out[name] = [_angle_deg(landmarks[chain[i]], landmarks[chain[i + 1]], landmarks[chain[i + 2]]) for i in range(3)]
    return out


def curl_ratios(angles: dict[str, list[float]]) -> dict[str, float]:
    """Normalise total finger flexion to 0 (open) .. 1 (fully closed)."""
    out: dict[str, float] = {}
    for name, triple in angles.items():
        lo, hi = CURL_RANGE_DEG[name]
        out[name] = float(np.clip((sum(triple) - lo) / (hi - lo), 0.0, 1.0))
    return out


def palm_frame(landmarks: np.ndarray, handedness: str) -> tuple[np.ndarray, np.ndarray]:
    """Right-handed palm frame: x = across palm, y = towards fingers, z = palm normal."""
    wrist = landmarks[WRIST]
    across = landmarks[INDEX_MCP] - landmarks[PINKY_MCP]
    forward = landmarks[MIDDLE_MCP] - wrist
    if handedness.lower().startswith("l"):
        across = -across

    x = _unit(across)
    z = _unit(np.cross(x, _unit(forward)))
    y = _unit(np.cross(z, x))
    rotation = np.column_stack((x, y, z))
    center = (wrist + landmarks[INDEX_MCP] + landmarks[PINKY_MCP]) / 3.0
    return rotation, center


def rotation_to_euler(r: np.ndarray) -> tuple[float, float, float]:
    """Intrinsic XYZ (roll, pitch, yaw) in degrees."""
    sy = float(np.sqrt(r[0, 0] ** 2 + r[1, 0] ** 2))
    if sy > 1e-6:
        roll = np.arctan2(r[2, 1], r[2, 2])
        pitch = np.arctan2(-r[2, 0], sy)
        yaw = np.arctan2(r[1, 0], r[0, 0])
    else:
        roll = np.arctan2(-r[1, 2], r[1, 1])
        pitch = np.arctan2(-r[2, 0], sy)
        yaw = 0.0
    return tuple(float(np.degrees(a)) for a in (roll, pitch, yaw))  # type: ignore[return-value]


def finger_spreads(landmarks: np.ndarray) -> dict[str, float]:
    """Abduction angle (deg) between neighbouring finger directions."""
    mcps = [1, INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP]
    tips = [4, 8, 12, 16, 20]
    dirs = [_unit(landmarks[t] - landmarks[m]) for m, t in zip(mcps, tips)]
    pairs = [("thumb_index", 0, 1), ("index_middle", 1, 2), ("middle_ring", 2, 3), ("ring_pinky", 3, 4)]
    out: dict[str, float] = {}
    for name, i, j in pairs:
        cos = float(np.clip(np.dot(dirs[i], dirs[j]), -1.0, 1.0))
        out[name] = float(np.degrees(np.arccos(cos)))
    return out


def build_pose(handedness: str, score: float, world: np.ndarray, pixels: np.ndarray) -> HandPose:
    angles = joint_angles(world)
    rotation, center = palm_frame(world, handedness)
    return HandPose(
        handedness=handedness,
        score=score,
        landmarks=world,
        pixels=pixels,
        joint_angles=angles,
        curls=curl_ratios(angles),
        spreads=finger_spreads(world),
        palm_rotation=rotation,
        palm_euler=rotation_to_euler(rotation),
        palm_normal=rotation[:, 2].copy(),
        palm_center=center,
    )
