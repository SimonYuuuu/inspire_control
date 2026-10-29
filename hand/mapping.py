"""Map a tracked :class:`HandPose` onto the hand's six degrees of freedom.

The two sides use opposite conventions: ``hand_pose.curls`` runs 0 (open) to 1
(closed), while the angle registers run 0 (flexed) to 1000 (extended). Everything
here is expressed as *extension* in 0..1 so the safety layer owns the only
conversion to raw register units.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .registers import DOF_ORDER

# Which hand_pose finger drives each flexion DOF. thumb_rotation is handled apart.
FLEXION_SOURCE: dict[str, str] = {
    "pinky": "pinky",
    "ring": "ring",
    "middle": "middle",
    "index": "index",
    "thumb_bend": "thumb",
}


@dataclass
class MappingConfig:
    """Operator-dependent calibration of the pose -> DOF mapping."""

    # thumb_index abduction angle (deg) spanning retracted .. fully opposed
    thumb_rotation_range_deg: tuple[float, float] = (15.0, 55.0)
    invert_thumb_rotation: bool = False
    # per-DOF rescaling of the curl signal, in case an operator cannot reach the extremes
    curl_range: dict[str, tuple[float, float]] = field(default_factory=dict)

    def curl_span(self, finger: str) -> tuple[float, float]:
        return self.curl_range.get(finger, (0.0, 1.0))


def _normalise(value: float, lo: float, hi: float) -> float:
    if hi - lo < 1e-6:
        return 0.0
    return min(1.0, max(0.0, (value - lo) / (hi - lo)))


def pose_to_extension(pose, config: MappingConfig | None = None) -> list[float]:
    """Return extension in 0..1 per DOF, ordered as :data:`DOF_ORDER`.

    1.0 means fully extended for the five flexion DOFs, and fully opposed
    (thumb across the palm) for ``thumb_rotation``.
    """
    cfg = config or MappingConfig()
    out: list[float] = []
    for dof in DOF_ORDER:
        if dof == "thumb_rotation":
            lo, hi = cfg.thumb_rotation_range_deg
            opposition = _normalise(pose.spreads.get("thumb_index", lo), lo, hi)
            out.append(1.0 - opposition if cfg.invert_thumb_rotation else opposition)
        else:
            finger = FLEXION_SOURCE[dof]
            lo, hi = cfg.curl_span(finger)
            out.append(1.0 - _normalise(pose.curls.get(finger, 0.0), lo, hi))
    return out
