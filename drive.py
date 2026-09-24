#!/usr/bin/env python3
import argparse
import time

from gpiozero import DigitalOutputDevice


DEFAULT_PINS = {
    "in1": 22,  # physical pin 15
    "in2": 23,  # physical pin 16
    "in3": 26,  # physical pin 37
    "in4": 20,  # physical pin 38
}

DEFAULT_INVERT_LEFT = False
DEFAULT_INVERT_RIGHT = True


class Motor:
    def __init__(self, a_pin: int, b_pin: int, invert: bool = False):
        self.a = DigitalOutputDevice(a_pin)
        self.b = DigitalOutputDevice(b_pin)
        self.invert = invert
        self.stop()

    def forward(self):
        if self.invert:
            self.backward_raw()
        else:
            self.forward_raw()

    def backward(self):
        if self.invert:
            self.forward_raw()
        else:
            self.backward_raw()

    def forward_raw(self):
        self.a.on()
        self.b.off()

    def backward_raw(self):
        self.a.off()
        self.b.on()

    def stop(self):
        self.a.off()
        self.b.off()


def stop_all(left: Motor, right: Motor):
    left.stop()
    right.stop()


def dance(left: Motor, right: Motor, total_seconds: float):
    step_seconds = 0.35
    end_time = time.monotonic() + total_seconds

    moves = [
        (left.forward, right.backward),
        (left.backward, right.forward),
        (left.forward, right.forward),
        (left.backward, right.backward),
        (left.forward, right.backward),
        (left.backward, right.forward),
    ]

    while time.monotonic() < end_time:
        for left_move, right_move in moves:
            if time.monotonic() >= end_time:
                return
            left_move()
            right_move()
            time.sleep(min(step_seconds, end_time - time.monotonic()))
            stop_all(left, right)
            time.sleep(min(0.08, max(0.0, end_time - time.monotonic())))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "command",
        choices=["forward", "backward", "left", "right", "stop", "dance"],
    )
    parser.add_argument("--seconds", type=float, default=0.5)
    parser.add_argument("--in1", type=int, default=DEFAULT_PINS["in1"])
    parser.add_argument("--in2", type=int, default=DEFAULT_PINS["in2"])
    parser.add_argument("--in3", type=int, default=DEFAULT_PINS["in3"])
    parser.add_argument("--in4", type=int, default=DEFAULT_PINS["in4"])
    parser.add_argument("--invert-left", action=argparse.BooleanOptionalAction, default=DEFAULT_INVERT_LEFT)
    parser.add_argument("--invert-right", action=argparse.BooleanOptionalAction, default=DEFAULT_INVERT_RIGHT)
    args = parser.parse_args()

    left = Motor(args.in1, args.in2, invert=args.invert_left)
    right = Motor(args.in3, args.in4, invert=args.invert_right)

    try:
        print(f"{args.command} for {args.seconds:.2f}s", flush=True)
        if args.command == "dance":
            dance(left, right, args.seconds)
            return
        elif args.command == "forward":
            left.forward()
            right.forward()
        elif args.command == "backward":
            left.backward()
            right.backward()
        elif args.command == "left":
            left.backward()
            right.forward()
        elif args.command == "right":
            left.forward()
            right.backward()
        elif args.command == "stop":
            pass
        time.sleep(args.seconds)
    finally:
        left.stop()
        right.stop()
        print("stopped", flush=True)


if __name__ == "__main__":
    main()
