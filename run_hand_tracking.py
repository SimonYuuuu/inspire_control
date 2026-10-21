#!/usr/bin/env python3
"""Live USB-camera hand tracking: finger joint angles + palm orientation.

Examples:
    python3 run_hand_tracking.py --list
    python3 run_hand_tracking.py --device 0
    python3 run_hand_tracking.py --device 0 --headless --log poses.jsonl
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import cv2

from camera import StreamReader, list_devices, open_camera
from hand_tracker import HandTracker
from visualize import render


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--device", default="0", help="camera index or video file path (default: 0)")
    p.add_argument("--list", action="store_true", help="list working camera devices and exit")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=int, default=30, help="requested camera rate")
    p.add_argument("--max-fps", type=float, help="throttle the input rate; stale frames are drained, not queued")
    p.add_argument("--max-hands", type=int, default=1)
    p.add_argument("--smoothing", type=float, default=0.5, help="0 = off, closer to 1 = smoother but laggier")
    p.add_argument("--no-flip", action="store_true", help="do not mirror the preview")
    p.add_argument("--headless", action="store_true", help="no window; print pose JSON to stdout")
    p.add_argument("--log", help="append per-frame pose JSON lines to this file")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    if args.list:
        devices = list_devices()
        if not devices:
            print("no working camera found; check `ls /dev/video*` and that the USB camera is plugged in")
            return 1
        print(f"{'index':<6} {'resolution':<12} {'fps':<6} name")
        for d in devices:
            print(f"{d['index']:<6} {d['width']}x{d['height']:<7} {d['fps']:<6} {d['name']}")
        return 0

    device: int | str = int(args.device) if args.device.isdigit() else args.device
    stream = StreamReader(open_camera(device, args.width, args.height, args.fps), args.max_fps)
    log_file = open(args.log, "a", encoding="utf-8") if args.log else None
    fps = 0.0
    latency_ms = 0.0
    last = time.perf_counter()

    try:
        with HandTracker(max_hands=args.max_hands, smoothing=args.smoothing) as tracker:
            start = time.monotonic()
            last_ts_ms = -1
            while True:
                grabbed = stream.read()
                if grabbed is None:
                    print("frame grab failed", file=sys.stderr)
                    break
                frame, captured_at = grabbed
                if not args.no_flip:
                    frame = cv2.flip(frame, 1)

                # VIDEO mode needs real, strictly increasing capture timestamps
                timestamp_ms = max(int((captured_at - start) * 1000.0), last_ts_ms + 1)
                last_ts_ms = timestamp_ms
                poses = tracker.process(frame, timestamp_ms)
                latency_ms = (time.monotonic() - captured_at) * 1000.0

                now = time.perf_counter()
                dt = now - last
                last = now
                if dt > 0:
                    fps = 0.9 * fps + 0.1 / dt if fps else 1.0 / dt

                if log_file or args.headless:
                    record = json.dumps(
                        {
                            "t": round(now, 4),
                            "latency_ms": round(latency_ms, 1),
                            "hands": [p.as_dict() for p in poses],
                        }
                    )
                    if log_file:
                        log_file.write(record + "\n")
                        log_file.flush()
                    if args.headless:
                        print(record, flush=True)

                if not args.headless:
                    cv2.imshow("hand_teleop", render(frame, poses, fps, latency_ms))
                    if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                        break
    except KeyboardInterrupt:
        pass
    finally:
        stream.release()
        if log_file:
            log_file.close()
        if not args.headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
