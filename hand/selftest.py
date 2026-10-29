#!/usr/bin/env python3
"""Hardware-free checks of the mapping and safety layers.

    python3 -m hand.selftest
"""

from __future__ import annotations

from .mapping import MappingConfig, pose_to_extension
from .registers import DOF_ORDER, RAW_MAX
from .safety import SafetyConfig, SafetyGovernor


class FakePose:
    def __init__(self, curl: float, spread: float) -> None:
        self.curls = {f: curl for f in ("thumb", "index", "middle", "ring", "pinky")}
        self.spreads = {"thumb_index": spread}


def check(name: str, condition: bool) -> None:
    if not condition:
        raise AssertionError(name)
    print(f"  ok  {name}")


def main() -> int:
    print("mapping")
    open_hand = pose_to_extension(FakePose(curl=0.0, spread=15.0), MappingConfig())
    closed = pose_to_extension(FakePose(curl=1.0, spread=55.0), MappingConfig())
    check("open hand -> full extension", all(abs(v - 1.0) < 1e-6 for v in open_hand[:5]))
    check("closed hand -> zero extension", all(abs(v) < 1e-6 for v in closed[:5]))
    check("thumb rotation follows abduction", open_hand[5] == 0.0 and closed[5] == 1.0)
    inverted = pose_to_extension(FakePose(curl=0.0, spread=55.0), MappingConfig(invert_thumb_rotation=True))
    check("thumb rotation invertible", inverted[5] == 0.0)
    out_of_range = pose_to_extension(FakePose(curl=5.0, spread=-90.0), MappingConfig())
    check("absurd pose stays in 0..1", all(0.0 <= v <= 1.0 for v in out_of_range))

    print("travel window")
    governor = SafetyGovernor(SafetyConfig(travel={"index": (600, 900)}, max_rate=1e9, deadband=0))
    governor.reset([RAW_MAX] * 6)
    commanded = governor.step([1.0] * 6, dt=1.0)
    check("unrestricted DOF reaches 1000", commanded[0] == RAW_MAX)
    check("restricted DOF capped at 900", commanded[DOF_ORDER.index("index")] == 900)
    commanded = governor.step([0.0] * 6, dt=1.0)
    check("restricted DOF floored at 600", commanded[DOF_ORDER.index("index")] == 600)
    check("every command inside hardware range", all(0 <= v <= RAW_MAX for v in commanded))

    inverted_window = SafetyGovernor(SafetyConfig(travel={"pinky": (900, 100)}, max_rate=1e9, deadband=0))
    inverted_window.reset([500] * 6)
    check("reversed window is normalised", inverted_window.step([0.0] * 6, 1.0)[0] == 100)

    print("slew limit")
    governor = SafetyGovernor(SafetyConfig(max_rate=500.0, deadband=0))
    governor.reset([RAW_MAX] * 6)
    step = governor.step([0.0] * 6, dt=0.02)  # one 50 Hz tick
    check("one tick moves at most rate*dt", all(RAW_MAX - v <= 10 for v in step))
    for _ in range(200):
        step = governor.step([0.0] * 6, dt=0.02)
    check("target is still reached eventually", all(v == 0 for v in step))

    print("deadband")
    governor = SafetyGovernor(SafetyConfig(max_rate=1e9, deadband=20))
    governor.reset([500] * 6)
    check("tremor is ignored", governor.step([0.51] * 6, 1.0) == [500] * 6)
    check("real motion passes", governor.step([0.8] * 6, 1.0) != [500] * 6)

    print("force guard")
    governor = SafetyGovernor(SafetyConfig(max_rate=1e9, deadband=0, force_limit=1000))
    governor.reset([500] * 6)
    overload = [1200, 0, 0, 0, 0, 0]
    held = governor.step([0.0] * 6, 1.0, forces=overload)
    check("overloaded DOF may not close further", held[0] == 500)
    check("other DOFs still move", held[1] == 0)
    check("overload is reported", governor.force_blocked[0] and not governor.force_blocked[1])
    check("overloaded DOF may still open", governor.step([1.0] * 6, 1.0, forces=overload)[0] == RAW_MAX)
    check("negative force counts too", governor.step([0.0] * 6, 1.0, forces=[-1200] + [0] * 5)[0] == RAW_MAX)

    print("stop poses")
    governor = SafetyGovernor(SafetyConfig(travel={"pinky": (200, 800)}))
    governor.reset([400] * 6)
    check("hold keeps the last command", governor.hold() == governor.commanded)
    check("release opens to the window top", governor.release()[0] == 800)

    print("\nall checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
