#!/usr/bin/env python3

import argparse
import json
import math
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


AXES = ("x", "y", "z", "rx", "ry", "rz")
POSITION_INDEX = {"x": 0, "y": 1, "z": 2}


def post_json(base_url: str, endpoint: str, payload=None, timeout: float = 2.0) -> Dict[str, Any]:
    body = json.dumps(payload or {}).encode("utf-8")
    request = Request(
        f"{base_url.rstrip('/')}/{endpoint.lstrip('/')}",
        data=body,
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        with urlopen(request, timeout=timeout) as response:
            result = json.loads(response.read())
    except HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"{endpoint} returned HTTP {exc.code}: {detail}") from exc
    except (URLError, TimeoutError, OSError) as exc:
        raise RuntimeError(f"{endpoint} request failed: {exc}") from exc
    if not isinstance(result, dict) or result.get("ok") is not True:
        raise RuntimeError(f"{endpoint} returned an unsuccessful response: {result!r}")
    return result


def parallel(callbacks):
    with ThreadPoolExecutor(max_workers=2) as executor:
        futures = {name: executor.submit(callback) for name, callback in callbacks.items()}
        results = {}
        for name, future in futures.items():
            try:
                results[name] = future.result()
            except Exception as exc:
                raise RuntimeError(f"{name} arm: {exc}") from exc
        return results


def read_poses(urls: Dict[str, str], timeout: float) -> Dict[str, List[float]]:
    states = parallel(
        {
            name: lambda selected=url: post_json(selected, "getstate", timeout=timeout)
            for name, url in urls.items()
        }
    )
    poses = {}
    for name, state in states.items():
        pose = state.get("pose")
        if not isinstance(pose, list) or len(pose) != 7:
            raise RuntimeError(f"{name} arm returned an invalid pose: {pose!r}")
        pose = [float(value) for value in pose]
        if not all(math.isfinite(value) for value in pose):
            raise RuntimeError(f"{name} arm pose contains NaN or Inf")
        if sum(value * value for value in pose[3:]) < 1e-18:
            raise RuntimeError(f"{name} arm returned a zero quaternion")
        poses[name] = pose
    return poses


def interpolate(starts, goals, alpha: float):
    return {
        name: [start + alpha * (goal - start) for start, goal in zip(starts[name], goals[name])]
        for name in starts
    }


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]


def offset_pose(pose, axis: str, distance: float, angle: float):
    target = pose.copy()
    if axis in POSITION_INDEX:
        target[POSITION_INDEX[axis]] += distance
        return target

    half_angle = angle / 2.0
    delta = [0.0, 0.0, 0.0, math.cos(half_angle)]
    delta[POSITION_INDEX[axis[1:]]] = math.sin(half_angle)
    target[3:] = quaternion_multiply(delta, pose[3:])
    return target


def send_poses(urls, poses, timeout: float):
    return parallel(
        {
            name: lambda selected=url, pose=poses[name]: post_json(
                selected, "pose", {"arr": pose}, timeout
            )
            for name, url in urls.items()
        }
    )


def move(urls, starts, goals, duration: float, rate: float, timeout: float) -> None:
    steps = max(1, round(duration * rate))
    started_at = time.monotonic()
    for step in range(1, steps + 1):
        send_poses(urls, interpolate(starts, goals, step / steps), timeout)
        remaining = started_at + step / rate - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)


def self_test() -> None:
    starts = {"left": [0.0] * 7, "right": [1.0] * 7}
    goals = {"left": [2.0] * 7, "right": [3.0] * 7}
    halfway = interpolate(starts, goals, 0.5)
    if halfway["left"] != [1.0] * 7 or halfway["right"] != [2.0] * 7:
        raise SystemExit(1)
    rotated = offset_pose([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], "rx", 0.0, math.pi / 2)
    expected = math.sqrt(0.5)
    if not math.isclose(rotated[3], expected) or not math.isclose(rotated[6], expected):
        raise SystemExit(1)


def main() -> None:
    parser = argparse.ArgumentParser(description="Dual-arm x/y/z/rx/ry/rz HTTP motion test")
    parser.add_argument("--left-url", default="http://127.0.0.1:5000")
    parser.add_argument("--right-url", default="http://127.0.0.1:5001")
    parser.add_argument("--distance", type=float, default=0.005, help="Relative motion in metres")
    parser.add_argument(
        "--angle-deg", type=float, default=3.0, help="Relative rx/ry/rz rotation in degrees"
    )
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--hold", type=float, default=0.5)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--execute", action="store_true", help="Actually move both arms")
    parser.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return
    if not math.isfinite(args.distance) or not 0.0 < abs(args.distance) <= 0.02:
        parser.error("--distance must be non-zero and at most 0.02 m")
    if not math.isfinite(args.angle_deg) or not 0.0 < abs(args.angle_deg) <= 10.0:
        parser.error("--angle-deg must be non-zero and at most 10 degrees")
    if args.duration < 0.2 or not 1.0 <= args.rate <= 100.0:
        parser.error("--duration must be >= 0.2 and --rate must be between 1 and 100 Hz")
    if not 0.0 <= args.hold <= 10.0 or args.timeout <= 0.0:
        parser.error("--hold must be 0..10 seconds and --timeout must be positive")

    urls = {"left": args.left_url, "right": args.right_url}
    starts = read_poses(urls, args.timeout)
    angle = math.radians(args.angle_deg)
    goals = {
        axis: {
            name: offset_pose(pose, axis, args.distance, angle)
            for name, pose in starts.items()
        }
        for axis in AXES
    }

    print(json.dumps({"start": starts, "targets": goals}, indent=2))
    if not args.execute:
        print("Preview only. Re-run with --execute to move both arms.")
        return

    for axis in AXES:
        print(f"Testing {axis}: moving both arms...")
        move(urls, starts, goals[axis], args.duration, args.rate, args.timeout)
        time.sleep(args.hold)
        move(urls, goals[axis], starts, args.duration, args.rate, args.timeout)
        time.sleep(args.hold)
        print(f"Testing {axis}: both arms returned to their initial poses.")


if __name__ == "__main__":
    main()
