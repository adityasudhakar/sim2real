#!/usr/bin/env python3
import argparse
import json
import subprocess
from pathlib import Path

import mujoco

from make_l_path_plan import reset_empty_start
from run_sandbox import MODEL_PATH, apply_motion_command


ROOT = Path(__file__).resolve().parent.parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, default=ROOT / "sim_l_path_plan.json")
    parser.add_argument("--out", type=Path, default=ROOT / "sim_l_path_plan.mp4")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--camera", default="overview")
    parser.add_argument("--trail-pause-s", type=float, default=0.7)
    args = parser.parse_args()

    plan_data = json.loads(args.plan.read_text())
    plan = plan_data["plan"]

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    reset_empty_start(model, data)

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    frame_interval = 1.0 / args.fps
    next_frame_time = 0.0

    ffmpeg_cmd = [
        "ffmpeg",
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{args.width}x{args.height}",
        "-r",
        str(args.fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(args.out),
    ]
    proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

    def maybe_write_frame():
        nonlocal next_frame_time
        while data.time + 1e-9 >= next_frame_time:
            renderer.update_scene(data, camera=args.camera)
            frame = renderer.render()
            proc.stdin.write(frame.tobytes())
            next_frame_time += frame_interval

    try:
        maybe_write_frame()
        for step in plan:
            command = step["command"]
            seconds = float(step["seconds"])
            start_time = data.time
            while data.time - start_time < seconds:
                apply_motion_command(model, data, command)
                mujoco.mj_step(model, data)
                maybe_write_frame()
            data.ctrl[:] = 0.0
            for _ in range(int(0.15 / model.opt.timestep)):
                mujoco.mj_step(model, data)
                maybe_write_frame()

        end_time = data.time + args.trail_pause_s
        while data.time < end_time:
            mujoco.mj_step(model, data)
            maybe_write_frame()
    finally:
        if proc.stdin:
            proc.stdin.close()
        proc.wait()
        renderer.close()

    if proc.returncode != 0:
        raise SystemExit(f"ffmpeg failed with exit code {proc.returncode}")
    print(args.out)


if __name__ == "__main__":
    main()
