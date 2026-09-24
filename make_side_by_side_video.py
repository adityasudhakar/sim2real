#!/usr/bin/env python3
import argparse
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", type=Path, default=Path("sim_l_path_plan.mp4"))
    parser.add_argument("--real", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("sim_real_side_by_side.mp4"))
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    filter_complex = (
        f"[0:v]scale=-2:{args.height},setsar=1[sim];"
        f"[1:v]scale=-2:{args.height},setsar=1[real];"
        "[sim][real]hstack=inputs=2[v]"
    )
    cmd = [
        "ffmpeg",
        "-y",
        "-i",
        str(args.sim),
        "-i",
        str(args.real),
        "-filter_complex",
        filter_complex,
        "-map",
        "[v]",
        "-an",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(args.out),
    ]
    subprocess.run(cmd, check=True)
    print(args.out)


if __name__ == "__main__":
    main()
