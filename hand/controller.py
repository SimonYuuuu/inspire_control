"""Real-time control thread between the tracker and the hand.

The vision loop never touches the socket. It only drops its latest result into
:meth:`HandController.submit`; a background thread runs the Modbus traffic on
its own fixed clock. That buys three things:

* a Modbus round-trip (or a stall, or a reconnect) cannot slow down tracking;
* the command rate stays constant even though the camera delivers ~28 fps;
* the watchdog keeps running when the vision loop is the thing that died.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field, replace
from enum import Enum

from .client import HandError, InspireHand
from .registers import DOF_COUNT
from .safety import SafetyGovernor


class StopAction(str, Enum):
    HOLD = "hold"  # freeze at the current physical pose
    OPEN = "open"  # retreat to the top of every travel window


class State(str, Enum):
    IDLE = "idle"  # connected, not following the operator
    ACTIVE = "active"
    STOPPED = "stopped"  # emergency stop latched
    FAULT = "fault"  # transport lost, trying to recover


@dataclass
class Telemetry:
    state: State = State.IDLE
    reason: str = ""
    commanded: list[int] = field(default_factory=lambda: [0] * DOF_COUNT)
    actual: list[int] = field(default_factory=lambda: [0] * DOF_COUNT)
    forces: list[int] = field(default_factory=lambda: [0] * DOF_COUNT)
    force_blocked: list[bool] = field(default_factory=lambda: [False] * DOF_COUNT)
    write_ms: float = 0.0
    loop_hz: float = 0.0


class HandController:
    def __init__(
        self,
        client: InspireHand,
        governor: SafetyGovernor,
        rate_hz: float = 50.0,
        watchdog_s: float = 0.3,
        stop_action: StopAction = StopAction.HOLD,
        telemetry_hz: float = 10.0,
    ) -> None:
        self._client = client
        self._governor = governor
        self._period = 1.0 / max(1.0, rate_hz)
        self._watchdog_s = watchdog_s
        self._stop_action = stop_action
        self._telemetry_period = 1.0 / max(1.0, telemetry_hz)

        # Reentrant: a SIGINT handler runs on the main thread and calls trip(),
        # which may land while that same thread is inside submit().
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._state = State.IDLE
        self._reason = ""
        self._sample: list[float] | None = None
        self._sample_at = 0.0
        self._telemetry = Telemetry()
        self._stop_written = False
        self._resync = False
        self._loop_hz = 0.0

    # ------------------------------------------------------------------ API

    def start(self) -> None:
        if self._thread is not None:
            return
        self._running.set()
        self._thread = threading.Thread(target=self._run, name="hand-control", daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        """Latch a stop, let it reach the hand, then tear the thread down."""
        self.trip("shutdown")
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._client.close()

    def submit(self, extension: list[float] | None) -> None:
        """Hand the newest tracker result to the control thread.

        ``None`` means no hand was seen this frame; the sample timestamp is not
        refreshed, so the watchdog will fire if that persists.
        """
        if extension is None:
            return
        with self._lock:
            self._sample = list(extension)
            self._sample_at = time.monotonic()

    def engage(self) -> None:
        with self._lock:
            if self._state is State.STOPPED:
                return
            self._state = State.ACTIVE
            self._reason = ""
            self._sample = None
            # Give the operator one watchdog window to get a hand in frame.
            self._sample_at = time.monotonic()
            self._resync = True

    def disengage(self) -> None:
        with self._lock:
            if self._state is State.ACTIVE:
                self._state = State.IDLE
                self._reason = ""

    def trip(self, reason: str) -> None:
        """Emergency stop. Idempotent and safe to call from any thread."""
        with self._lock:
            if self._state is State.STOPPED:
                return
            self._state = State.STOPPED
            self._reason = reason
            self._stop_written = False

    def clear(self) -> None:
        """Release the latch. The hand stays idle until re-engaged."""
        with self._lock:
            if self._state is State.STOPPED:
                self._state = State.IDLE
                self._reason = ""
                self._sample = None

    @property
    def telemetry(self) -> Telemetry:
        with self._lock:
            return replace(self._telemetry)

    # --------------------------------------------------------------- thread

    def _run(self) -> None:
        next_tick = time.monotonic()
        # Backdate so the first dt is one nominal period, not a few microseconds.
        last_tick = next_tick - self._period
        last_telemetry = 0.0
        self._loop_hz = 0.0

        while self._running.is_set():
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(self._period, next_tick - now))
                continue
            # When a tick runs late, resync to now instead of letting the
            # backlog fire a burst of catch-up writes.
            next_tick = max(now + self._period, next_tick + self._period)
            dt = now - last_tick
            last_tick = now
            if dt > 0:
                self._loop_hz = 0.9 * self._loop_hz + 0.1 / dt if self._loop_hz else 1.0 / dt

            poll = now - last_telemetry >= self._telemetry_period
            if poll:
                last_telemetry = now

            try:
                self._tick(now, dt, poll)
            except HandError as exc:
                self._enter_fault(str(exc))

        self._final_stop()

    def _tick(self, now: float, dt: float, poll: bool) -> None:
        with self._lock:
            state = self._state
            sample = self._sample
            resync = self._resync
            age = now - self._sample_at if self._sample_at else float("inf")

        if state is State.FAULT:
            self._client.connect()
            self._governor.reset(self._client.read_angles())
            with self._lock:
                self._state = State.IDLE
                self._reason = ""
            self._publish()
            return

        if state is State.ACTIVE and age > self._watchdog_s:
            self.trip(f"watchdog: no hand for {age * 1000:.0f} ms")
            state = State.STOPPED

        if state is State.STOPPED:
            self._apply_stop()
            self._publish()
            return

        if resync:
            self._governor.reset(self._client.read_angles())
            with self._lock:
                self._resync = False

        forces = self._client.read_forces() if poll else None

        if state is State.ACTIVE and sample is not None:
            command = self._governor.step(sample, dt, forces)
        else:
            command = self._governor.hold()

        started = time.monotonic()
        self._client.set_angles(list(command))
        write_ms = (time.monotonic() - started) * 1000.0

        self._publish(forces, self._client.read_angles() if poll else None, write_ms)

    def _publish(
        self,
        forces: list[int] | None = None,
        actual: list[int] | None = None,
        write_ms: float | None = None,
    ) -> None:
        with self._lock:
            previous = self._telemetry
            self._telemetry = Telemetry(
                state=self._state,
                reason=self._reason,
                commanded=self._governor.commanded,
                actual=actual if actual is not None else previous.actual,
                forces=forces if forces is not None else previous.forces,
                force_blocked=list(self._governor.force_blocked),
                write_ms=previous.write_ms if write_ms is None else write_ms,
                loop_hz=self._loop_hz,
            )

    def _apply_stop(self) -> None:
        """Write the stop pose once, then stop touching the angle registers."""
        if self._stop_written:
            return
        if self._stop_action is StopAction.OPEN:
            command = self._governor.release()
        else:
            # Writing the measured pose back as the setpoint halts motion where
            # the fingers physically are, not where the last target wanted them.
            actual = self._client.read_angles()
            self._governor.reset(actual)
            command = self._governor.hold()
        self._client.set_angles(list(command))
        self._stop_written = True

    def _enter_fault(self, reason: str) -> None:
        with self._lock:
            self._state = State.FAULT
            self._reason = reason
            self._telemetry.state = State.FAULT
            self._telemetry.reason = reason
        self._client.close()
        time.sleep(0.3)

    def _final_stop(self) -> None:
        try:
            self._apply_stop()
        except HandError:
            pass
