#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path

from drive import DEFAULT_INVERT_LEFT, DEFAULT_INVERT_RIGHT, DEFAULT_PINS, Motor, stop_all


COMMANDS = {"forward", "backward", "left", "right", "turn_left", "turn_right", "stop"}


def run_command(left, right, command, seconds):
    normalized = {
        "turn_left": "left",
        "turn_right": "right",
    }.get(command, command)

    print(f"{normalized} for {seconds:.2f}s", flush=True)
    if normalized == "forward":
        left.forward()
        right.forward()
    elif normalized == "backward":
        left.backward()
        right.backward()
    elif normalized == "left":
        left.backward()
        right.forward()
    elif normalized == "right":
        left.forward()
        right.backward()
    elif normalized == "stop":
        pass
    else:
        raise ValueError(f"Unsupported command: {command}")

    time.sleep(seconds)
    stop_all(left, right)
    time.sleep(0.15)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("plan_json", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--in1", type=int, default=DEFAULT_PINS["in1"])
    parser.add_argument("--in2", type=int, default=DEFAULT_PINS["in2"])
    parser.add_argument("--in3", type=int, default=DEFAULT_PINS["in3"])
    parser.add_argument("--in4", type=int, default=DEFAULT_PINS["in4"])
    parser.add_argument(
        "--invert-left",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_INVERT_LEFT,
    )
    parser.add_argument(
        "--invert-right",
        action=argparse.BooleanOptionalAction,
        default=DEFAULT_INVERT_RIGHT,
    )
    args = parser.parse_args()

    plan_data = json.loads(args.plan_json.read_text())
    plan = plan_data.get("plan", [])
    if not plan:
        raise SystemExit("Plan JSON does not contain a non-empty 'plan' list.")

    for step in plan:
        command = step.get("command")
        seconds = step.get("seconds")
        if command not in COMMANDS:
            raise SystemExit(f"Invalid command in plan: {command!r}")
        if not isinstance(seconds, (int, float)) or seconds < 0:
            raise SystemExit(f"Invalid seconds in plan step: {step!r}")

    print("Plan:")
    for step in plan:
        print(f"  {step['command']} {step['seconds']:.2f}s")

    if args.dry_run:
        return

    input("Place rover at the start pose, then press Enter to execute plan...")

    left = Motor(args.in1, args.in2, invert=args.invert_left)
    right = Motor(args.in3, args.in4, invert=args.invert_right)
    try:
        for step in plan:
            run_command(left, right, step["command"], float(step["seconds"]))
    finally:
        stop_all(left, right)
        print("stopped", flush=True)


if __name__ == "__main__":
    main()
