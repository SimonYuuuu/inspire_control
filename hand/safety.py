"""Turn desired extensions into register values the hand can safely execute.

Three independent guards, applied in order:

1. travel window  - never command outside the hardware's 0..1000 range, or
   outside a tighter per-DOF window the operator configured;
2. force guard    - a DOF already pushing harder than ``force_limit`` may open
   but may not close further, so a jammed finger cannot keep loading up;
3. slew limit     - bound how far any DOF may move per control tick, so a
   tracking glitch becomes a slow drift instead of a full-speed slam.

A deadband on top of that keeps the motors quiet when the operator's hand is
merely trembling.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .registers import DOF_COUNT, DOF_ORDER, RAW_MAX, RAW_MIN


@dataclass
class SafetyConfig:
    travel: dict[str, tuple[int, int]] = field(default_factory=dict)
    max_rate: float = 900.0  # register units per second; full sweep in ~1.1 s
    deadband: int = 6  # register units
    force_limit: int = 1500  # grams, absolute

    def window(self, dof: str) -> tuple[int, int]:
        lo, hi = self.travel.get(dof, (RAW_MIN, RAW_MAX))
        lo = max(RAW_MIN, min(RAW_MAX, int(lo)))
        hi = max(RAW_MIN, min(RAW_MAX, int(hi)))
        return (lo, hi) if lo <= hi else (hi, lo)


class SafetyGovernor:
    """Stateful filter between the mapping layer and the angle registers."""

    def __init__(self, config: SafetyConfig | None = None) -> None:
        self.config = config or SafetyConfig()
        self._windows = [self.config.window(dof) for dof in DOF_ORDER]
        self._commanded: list[int] = [hi for _, hi in self._windows]
        self.force_blocked: list[bool] = [False] * DOF_COUNT

    @property
    def commanded(self) -> list[int]:
        return list(self._commanded)

    def reset(self, current: list[int] | None = None) -> None:
        """Re-anchor on the hand's actual pose so the next tick does not jump."""
        if current is None:
            self._commanded = [hi for _, hi in self._windows]
        else:
            self._commanded = [
                max(lo, min(hi, int(v))) for v, (lo, hi) in zip(current, self._windows)
            ]
        self.force_blocked = [False] * DOF_COUNT

    def step(self, extension: list[float], dt: float, forces: list[int] | None = None) -> list[int]:
        """Advance the command vector one control tick toward ``extension``."""
        max_delta = max(1.0, self.config.max_rate * max(dt, 0.0))
        out: list[int] = []
        for i, dof_extension in enumerate(extension):
            lo, hi = self._windows[i]
            previous = self._commanded[i]

            target = lo + (hi - lo) * min(1.0, max(0.0, dof_extension))

            blocked = forces is not None and abs(forces[i]) > self.config.force_limit
            self.force_blocked[i] = blocked
            if blocked:
                target = max(target, previous)

            if abs(target - previous) < self.config.deadband:
                out.append(previous)
                continue

            delta = max(-max_delta, min(max_delta, target - previous))
            out.append(int(round(max(lo, min(hi, previous + delta)))))

        self._commanded = out
        return list(out)

    def hold(self) -> list[int]:
        """Command vector that keeps the hand exactly where it was told to be."""
        return list(self._commanded)

    def release(self) -> list[int]:
        """Command vector that opens every DOF to the top of its travel window."""
        self._commanded = [hi for _, hi in self._windows]
        return list(self._commanded)
