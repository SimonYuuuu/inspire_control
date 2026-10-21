"""Overlay drawing for hand poses (skeleton, palm axes, per-finger curl bars)."""

from __future__ import annotations

import cv2
import numpy as np

from hand_pose import FINGERS, HandPose

CONNECTIONS: tuple[tuple[int, int], ...] = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17),
)

FINGER_COLORS = {
    "thumb": (0, 165, 255),
    "index": (0, 255, 0),
    "middle": (255, 255, 0),
    "ring": (255, 0, 255),
    "pinky": (0, 0, 255),
}


def draw_skeleton(frame: np.ndarray, pose: HandPose) -> None:
    pts = pose.pixels.astype(int)
    for a, b in CONNECTIONS:
        cv2.line(frame, tuple(pts[a]), tuple(pts[b]), (200, 200, 200), 2, cv2.LINE_AA)
    for idx, (x, y) in enumerate(pts):
        color = (255, 255, 255) if idx == 0 else FINGER_COLORS[FINGERS[(idx - 1) // 4]]
        cv2.circle(frame, (x, y), 4, color, -1, cv2.LINE_AA)


def draw_palm_axes(frame: np.ndarray, pose: HandPose, length_px: float = 70.0) -> None:
    """Project the palm frame onto the image using the 2D wrist/MCP triangle as anchor."""
    origin = pose.pixels[[0, 5, 17]].mean(axis=0)
    axes_px = pose.palm_rotation[:2, :].T  # rows: x, y, z axis -> (u, v) components
    colors = ((0, 0, 255), (0, 255, 0), (255, 128, 0))
    for axis, color in zip(axes_px, colors):
        direction = np.array([axis[0], -axis[1]])
        norm = float(np.linalg.norm(direction))
        if norm < 1e-6:
            continue
        end = origin + direction / norm * length_px
        cv2.arrowedLine(frame, tuple(origin.astype(int)), tuple(end.astype(int)), color, 2, cv2.LINE_AA, tipLength=0.2)


def draw_hud(frame: np.ndarray, pose: HandPose, slot: int = 0) -> None:
    x0 = 10 + slot * 230
    y0 = 24
    roll, pitch, yaw = pose.palm_euler
    header = f"{pose.handedness} ({pose.score:.2f})"
    cv2.putText(frame, header, (x0, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(
        frame,
        f"rpy {roll:6.1f} {pitch:6.1f} {yaw:6.1f}",
        (x0, y0 + 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (180, 220, 255),
        1,
        cv2.LINE_AA,
    )
    for i, finger in enumerate(FINGERS):
        curl = pose.curls[finger]
        y = y0 + 48 + i * 22
        cv2.putText(frame, f"{finger[:5]:<6}", (x0, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
        bar_x = x0 + 56
        cv2.rectangle(frame, (bar_x, y), (bar_x + 140, y + 12), (70, 70, 70), -1)
        cv2.rectangle(frame, (bar_x, y), (bar_x + int(140 * curl), y + 12), FINGER_COLORS[finger], -1)
        cv2.putText(
            frame,
            f"{curl:4.2f}",
            (bar_x + 146, y + 11),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )


def render(frame: np.ndarray, poses: list[HandPose], fps: float | None = None, latency_ms: float | None = None) -> np.ndarray:
    for slot, pose in enumerate(poses):
        draw_skeleton(frame, pose)
        draw_palm_axes(frame, pose)
        draw_hud(frame, pose, slot)
    if fps is not None:
        cv2.putText(
            frame,
            f"{fps:5.1f} FPS",
            (frame.shape[1] - 110, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    if latency_ms is not None:
        cv2.putText(
            frame,
            f"{latency_ms:5.0f} ms",
            (frame.shape[1] - 110, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    if not poses:
        cv2.putText(frame, "no hand", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
    return frame
