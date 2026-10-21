#!/usr/bin/env python3
"""Camera-free sanity check of the pose geometry (needs only numpy)."""

from __future__ import annotations

import numpy as np

from hand_pose import FINGER_CHAINS, build_pose


def synthetic_hand(curl: float) -> np.ndarray:
    """Flat right hand in the XY plane, fingers bent by ``curl`` * 90 deg per joint."""
    lm = np.zeros((21, 3))
    bases = {"thumb": -0.035, "index": -0.015, "middle": 0.0, "ring": 0.015, "pinky": 0.03}
    for finger, chain in FINGER_CHAINS.items():
        x = bases[finger]
        pos = np.array([x, 0.03, 0.0])
        direction = np.array([0.0, 1.0, 0.0])
        lm[chain[1]] = pos
        for j in range(2, 5):
            angle = np.radians(curl * 80.0)
            c, s = np.cos(angle), np.sin(angle)
            direction = np.array([direction[0], direction[1] * c, -s * abs(direction[1]) + direction[2] * c])
            direction /= np.linalg.norm(direction)
            pos = pos + direction * 0.03
            lm[chain[j]] = pos
    lm[0] = np.array([0.0, -0.02, 0.0])
    return lm


def main() -> int:
    for curl in (0.0, 0.5, 1.0):
        lm = synthetic_hand(curl)
        pose = build_pose("Right", 1.0, lm, lm[:, :2] * 1000 + 300)
        print(f"curl={curl:.1f} -> {pose.as_dict()['curls']}  palm_euler={pose.as_dict()['palm_euler']}")

    open_hand = build_pose("Right", 1.0, synthetic_hand(0.0), np.zeros((21, 2)))
    closed_hand = build_pose("Right", 1.0, synthetic_hand(1.0), np.zeros((21, 2)))
    assert all(closed_hand.curls[f] >= open_hand.curls[f] for f in open_hand.curls), "curl must increase when bending"
    assert np.allclose(np.linalg.det(open_hand.palm_rotation), 1.0, atol=1e-6), "palm frame must be a rotation"
    print("ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
