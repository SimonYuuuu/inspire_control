#!/usr/bin/env python3
"""Teleoperate an Inspire RH56DFTP from a USB camera.

    python3 -m hand.teleop --dry-run                 # no hardware, verify mapping
    python3 -m hand.teleop --host 192.168.123.211    # live

Control starts disengaged. Press SPACE to follow your hand, ESC to stop.

SAFETY: this is a software stop over TCP, not a safety-rated e-stop. Keep the
24 V supply switch within reach; that is the only stop that works when the
network, the process, or the PC is the thing that failed.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import cv2

from camera import StreamReader, list_devices, open_camera
from hand_tracker import HandTracker
from visualize import render

from .client import HandError, InspireHand, SimulatedHand
from .controller import HandController, StopAction, State
from .mapping import MappingConfig, pose_to_extension
from .registers import DOF_ORDER
from .safety import SafetyConfig, SafetyGovernor

_STATE_COLOUR = {
    State.IDLE: (180, 180, 180),
    State.ACTIVE: (80, 220, 80),
    State.STOPPED: (60, 60, 240),
    State.FAULT: (0, 165, 255),
}


def parse_limit(spec: str) -> tuple[str, tuple[int, int]]:
    name, _, span = spec.partition("=")
    lo, _, hi = span.partition(":")
    if name not in DOF_ORDER or not lo or not hi:
        raise argparse.ArgumentTypeError(f"expected <dof>=<lo>:<hi> with dof in {DOF_ORDER}, got {spec!r}")
    return name, (int(lo), int(hi))


def parse_range(spec: str) -> tuple[float, float]:
    lo, _, hi = spec.partition(":")
    if not lo or not hi:
        raise argparse.ArgumentTypeError(f"expected <lo>:<hi>, got {spec!r}")
    return float(lo), float(hi)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)

    hw = p.add_argument_group("hand")
    hw.add_argument("--host", default="192.168.123.211")
    hw.add_argument("--port", type=int, default=6000)
    hw.add_argument("--dry-run", action="store_true", help="simulate the hand; never opens a socket")
    hw.add_argument("--rate", type=float, default=50.0, help="control loop rate in Hz")

    sf = p.add_argument_group("safety")
    sf.add_argument("--limit", type=parse_limit, action="append", default=[],
                    help="restrict one DOF, e.g. --limit thumb_bend=600:1000 (repeatable)")
    sf.add_argument("--max-rate", type=float, default=900.0, help="slew limit in register units per second")
    sf.add_argument("--deadband", type=int, default=6, help="ignore commanded changes smaller than this")
    sf.add_argument("--force-limit", type=int, default=1500, help="grams above which a DOF may no longer close")
    sf.add_argument("--watchdog-ms", type=float, default=300.0, help="stop if no hand is seen for this long")
    sf.add_argument("--stop-action", choices=[s.value for s in StopAction], default=StopAction.HOLD.value)

    mp = p.add_argument_group("mapping")
    mp.add_argument("--thumb-rot-range", type=parse_range, default=(15.0, 55.0),
                    help="thumb-index abduction in degrees spanning retracted:opposed")
    mp.add_argument("--invert-thumb-rot", action="store_true")
    mp.add_argument("--follow", choices=["Left", "Right", "any"], default="any", help="which hand to obey")

    cam = p.add_argument_group("camera")
    cam.add_argument("--device", default="0")
    cam.add_argument("--list", action="store_true", help="list working camera devices and exit")
    cam.add_argument("--width", type=int, default=1280)
    cam.add_argument("--height", type=int, default=720)
    cam.add_argument("--fps", type=int, default=30)
    cam.add_argument("--max-fps", type=float)
    cam.add_argument("--smoothing", type=float, default=0.5)
    cam.add_argument("--no-flip", action="store_true")
    cam.add_argument("--headless", action="store_true", help="no window; status goes to stdout")
    cam.add_argument("--auto-engage", action="store_true",
                     help="start following immediately; required for headless motion")

    return p.parse_args()


def pick_pose(poses, follow: str):
    if not poses:
        return None
    if follow == "any":
        return poses[0]
    for pose in poses:
        if pose.handedness == follow:
            return pose
    return None


def draw_status(frame, telemetry, engaged_hint: str):
    height = frame.shape[0]
    colour = _STATE_COLOUR.get(telemetry.state, (255, 255, 255))
    banner = telemetry.state.value.upper()
    if telemetry.reason:
        banner += f"  ({telemetry.reason})"
    cv2.rectangle(frame, (0, height - 86), (frame.shape[1], height), (0, 0, 0), -1)
    cv2.putText(frame, banner, (10, height - 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)

    cells = []
    for i, dof in enumerate(DOF_ORDER):
        mark = "!" if telemetry.force_blocked[i] else ""
        cells.append(f"{dof[:5]}:{telemetry.commanded[i]:>4}{mark}")
    cv2.putText(frame, " ".join(cells), (10, height - 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
    cv2.putText(
        frame,
        f"{telemetry.loop_hz:4.0f} Hz  write {telemetry.write_ms:4.1f} ms   {engaged_hint}",
        (10, height - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (160, 160, 160),
        1,
    )
    return frame


def main() -> int:
    args = parse_args()

    if args.list:
        for d in list_devices():
            print(f"{d['index']:<4} {d['width']}x{d['height']:<8} {d['fps']:<5} {d['name']}")
        return 0

    governor = SafetyGovernor(
        SafetyConfig(
            travel=dict(args.limit),
            max_rate=args.max_rate,
            deadband=args.deadband,
            force_limit=args.force_limit,
        )
    )
    mapping = MappingConfig(
        thumb_rotation_range_deg=args.thumb_rot_range,
        invert_thumb_rotation=args.invert_thumb_rot,
    )

    client = SimulatedHand() if args.dry_run else InspireHand(args.host, args.port)
    try:
        client.connect()
        governor.reset(client.read_angles())
    except HandError as exc:
        print(f"hand not reachable: {exc}", file=sys.stderr)
        return 1

    controller = HandController(
        client,
        governor,
        rate_hz=args.rate,
        watchdog_s=args.watchdog_ms / 1000.0,
        stop_action=StopAction(args.stop_action),
    )
    controller.start()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda _num, _frame, name=sig.name: controller.trip(f"signal {name}"))

    device: int | str = int(args.device) if args.device.isdigit() else args.device
    stream = StreamReader(open_camera(device, args.width, args.height, args.fps), args.max_fps)

    if args.auto_engage:
        controller.engage()

    fps = 0.0
    last = time.perf_counter()
    try:
        with HandTracker(max_hands=2 if args.follow != "any" else 1, smoothing=args.smoothing) as tracker:
            start = time.monotonic()
            last_ts_ms = -1
            while True:
                grabbed = stream.read()
                if grabbed is None:
                    controller.trip("camera stopped delivering frames")
                    break
                frame, captured_at = grabbed
                if not args.no_flip:
                    frame = cv2.flip(frame, 1)

                timestamp_ms = max(int((captured_at - start) * 1000.0), last_ts_ms + 1)
                last_ts_ms = timestamp_ms
                poses = tracker.process(frame, timestamp_ms)

                pose = pick_pose(poses, args.follow)
                controller.submit(pose_to_extension(pose, mapping) if pose is not None else None)

                now = time.perf_counter()
                dt = now - last
                last = now
                if dt > 0:
                    fps = 0.9 * fps + 0.1 / dt if fps else 1.0 / dt

                telemetry = controller.telemetry
                if args.headless:
                    print(
                        f"{telemetry.state.value:<7} cmd={telemetry.commanded} "
                        f"act={telemetry.actual} {telemetry.loop_hz:.0f}Hz {telemetry.reason}",
                        flush=True,
                    )
                    continue

                view = draw_status(
                    render(frame, poses, fps, (time.monotonic() - captured_at) * 1000.0),
                    telemetry,
                    "SPACE engage  ESC stop  c clear  q quit",
                )
                cv2.imshow("inspire teleop", view)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    controller.trip("operator stop")
                elif key == ord(" "):
                    controller.disengage() if telemetry.state is State.ACTIVE else controller.engage()
                elif key == ord("c"):
                    controller.clear()
                elif key == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        stream.release()
        if not args.headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
