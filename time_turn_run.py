#!/usr/bin/env python3
import argparse
import time

from drive import DEFAULT_INVERT_LEFT, DEFAULT_INVERT_RIGHT, DEFAULT_PINS, Motor, stop_all


def run_turn(left, right, direction):
    if direction == "left":
        left.backward()
        right.forward()
    elif direction == "right":
        left.forward()
        right.backward()
    else:
        raise ValueError(direction)


def main():
    parser = argparse.ArgumentParser(
        description="Turn until Enter is pressed, then print elapsed time."
    )
    parser.add_argument("direction", choices=["left", "right"], nargs="?", default="right")
    parser.add_argument("--degrees", type=float, default=90.0)
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

    left = Motor(args.in1, args.in2, invert=args.invert_left)
    right = Motor(args.in3, args.in4, invert=args.invert_right)

    print(f"Mark a {args.degrees:.1f} degree {args.direction} turn reference.")
    input("Place rover at the start heading, then press Enter to start...")

    start = time.monotonic()
    try:
        run_turn(left, right, args.direction)
        input(f"Press Enter the moment the rover has turned {args.degrees:.1f} degrees...")
    finally:
        elapsed = time.monotonic() - start
        stop_all(left, right)

    angular_speed_deg_s = args.degrees / elapsed if elapsed > 0 else 0.0
    print("stopped")
    print(f"elapsed_s: {elapsed:.3f}")
    print(f"degrees: {args.degrees:.1f}")
    print(f"angular_speed_deg_s: {angular_speed_deg_s:.2f}")


if __name__ == "__main__":
    main()
