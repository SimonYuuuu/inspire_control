"""Modbus holding-register map for the Inspire RH56DFTP.

Only addresses actually exercised against the hand are listed here. The RH56E2
documents floating around online also describe error/status blocks at 1606/1612,
but reading those as 16-bit registers on this hand returns misaligned garbage
(0xFFFF / 10278), so they are deliberately absent.
"""

from __future__ import annotations

DOF_ORDER = ("pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotation")
DOF_COUNT = len(DOF_ORDER)

ANGLE_SET = 1486  # W (FC16): target angle per DOF
ANGLE_ACT = 1546  # R (FC03): actual angle per DOF
FORCE_ACT = 1582  # R (FC03): gripping force per DOF, signed int16 grams
SPEED_SET = 1522  # W (FC16): documented, never written by this package

RAW_MIN = 0  # fully flexed / thumb retracted
RAW_MAX = 1000  # fully extended / thumb across palm
NO_ACTION = 0xFFFF  # -1: leave this DOF untouched

TACTILE_REGIONS: tuple[tuple[str, int, int, tuple[int, int]], ...] = (
    ("pinky_tip", 3000, 9, (3, 3)),
    ("pinky_nail", 3018, 96, (12, 8)),
    ("pinky_pad", 3210, 80, (10, 8)),
    ("ring_tip", 3370, 9, (3, 3)),
    ("ring_nail", 3388, 96, (12, 8)),
    ("ring_pad", 3580, 80, (10, 8)),
    ("middle_tip", 3740, 9, (3, 3)),
    ("middle_nail", 3758, 96, (12, 8)),
    ("middle_pad", 3950, 80, (10, 8)),
    ("index_tip", 4110, 9, (3, 3)),
    ("index_nail", 4128, 96, (12, 8)),
    ("index_pad", 4320, 80, (10, 8)),
    ("thumb_tip", 4480, 9, (3, 3)),
    ("thumb_nail", 4498, 96, (12, 8)),
    ("thumb_middle", 4690, 9, (3, 3)),
    ("thumb_pad", 4708, 96, (12, 8)),
    ("palm", 4900, 112, (8, 14)),
)
