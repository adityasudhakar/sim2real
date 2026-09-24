#!/usr/bin/env python3
import argparse
import time

from drive import DEFAULT_INVERT_LEFT, DEFAULT_INVERT_RIGHT, DEFAULT_PINS, Motor


INCH_TO_M = 0.0254


def main():
    parser = argparse.ArgumentParser(
        description="Drive until Enter is pressed, then print elapsed time and speed."
    )
    parser.add_argument(
        "command",
        choices=["forward", "backward"],
        nargs="?",
        default="forward",
        help="Straight-line command to run.",
    )
    parser.add_argument("--distance-in", type=float, default=49.0)
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

    distance_m = args.distance_in * INCH_TO_M
    left = Motor(args.in1, args.in2, invert=args.invert_left)
    right = Motor(args.in3, args.in4, invert=args.invert_right)

    print(f"Mark a straight {args.distance_in:.1f} in / {distance_m:.3f} m lane.")
    input("Place rover nose/tape at the start line, then press Enter to start...")

    start = time.monotonic()
    try:
        if args.command == "forward":
            left.forward()
            right.forward()
        else:
            left.backward()
            right.backward()

        input("Press Enter the moment the rover reaches the finish line...")
    finally:
        elapsed = time.monotonic() - start
        left.stop()
        right.stop()

    speed_m_s = distance_m / elapsed if elapsed > 0 else 0.0
    print("stopped")
    print(f"elapsed_s: {elapsed:.3f}")
    print(f"distance_m: {distance_m:.3f}")
    print(f"speed_m_s: {speed_m_s:.3f}")


if __name__ == "__main__":
    main()
