#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from run_sandbox import MODEL_PATH, apply_motion_command, body_xy, rover_yaw


ROOT = Path(__file__).resolve().parent.parent


def yaw_quat(yaw_rad):
    return np.array([math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)])


def set_rover_pose(model, data, xy=(0.0, 0.0), yaw_deg=0.0):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rover_free")
    qpos_adr = model.jnt_qposadr[joint_id]
    data.qpos[qpos_adr : qpos_adr + 3] = [xy[0], xy[1], 0.0]
    data.qpos[qpos_adr + 3 : qpos_adr + 7] = yaw_quat(math.radians(yaw_deg))


def reset_empty_start(model, data):
    mujoco.mj_resetData(model, data)
    set_rover_pose(model, data)
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def settle(model, data, seconds=0.15):
    data.ctrl[:] = 0.0
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)


def step_command(model, data, command, seconds):
    steps = int(seconds / model.opt.timestep)
    for _ in range(steps):
        apply_motion_command(model, data, command)
        mujoco.mj_step(model, data)
    settle(model, data)


def run_candidate(model, data, first_forward_s, turn_s, second_forward_s):
    reset_empty_start(model, data)
    commands = [
        ("forward", first_forward_s),
        ("turn_right", turn_s),
        ("forward", second_forward_s),
    ]
    for command, seconds in commands:
        step_command(model, data, command, seconds)
    end_xy = body_xy(model, data, "rover")
    end_yaw = rover_yaw(model, data)
    return commands, end_xy, math.degrees(end_yaw)


def run_turn_candidate(model, data, turn_s):
    reset_empty_start(model, data)
    step_command(model, data, "turn_right", turn_s)
    return math.degrees(rover_yaw(model, data))


def frange(start, stop, step):
    value = start
    while value <= stop + 1e-9:
        yield round(value, 3)
        value += step


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--first-forward-m", type=float, default=0.6)
    parser.add_argument("--second-forward-m", type=float, default=0.6)
    parser.add_argument("--turn-deg", type=float, default=-90.0)
    parser.add_argument("--out", type=Path, default=ROOT / "sim_l_path_plan.json")
    parser.add_argument("--forward-speed-m-s", type=float, default=0.3615)
    parser.add_argument("--coarse-step", type=float, default=0.05)
    parser.add_argument("--fine-step", type=float, default=0.02)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    target_xy = np.array([args.second_forward_m, args.first_forward_m])
    target_yaw = args.turn_deg

    first_nominal = args.first_forward_m / args.forward_speed_m_s
    second_nominal = args.second_forward_m / args.forward_speed_m_s
    shared_forward_time = abs(args.first_forward_m - args.second_forward_m) < 1e-9
    candidate_count = 0

    best_turn = None
    for turn_s in frange(0.2, 2.2, args.coarse_step):
        candidate_count += 1
        yaw_deg = run_turn_candidate(model, data, turn_s)
        yaw_error = abs(((yaw_deg - target_yaw + 180.0) % 360.0) - 180.0)
        candidate = {"turn_s": turn_s, "yaw_deg": yaw_deg, "yaw_error_deg": yaw_error}
        if best_turn is None or candidate["yaw_error_deg"] < best_turn["yaw_error_deg"]:
            best_turn = candidate

    for turn_s in frange(
        max(0.1, best_turn["turn_s"] - 0.1),
        best_turn["turn_s"] + 0.1,
        args.fine_step,
    ):
        candidate_count += 1
        yaw_deg = run_turn_candidate(model, data, turn_s)
        yaw_error = abs(((yaw_deg - target_yaw + 180.0) % 360.0) - 180.0)
        candidate = {"turn_s": turn_s, "yaw_deg": yaw_deg, "yaw_error_deg": yaw_error}
        if candidate["yaw_error_deg"] < best_turn["yaw_error_deg"]:
            best_turn = candidate

    best_plan = None
    first_times = list(frange(max(0.2, first_nominal - 0.5), first_nominal + 0.5, 0.1))
    second_times = [None] if shared_forward_time else list(
        frange(max(0.2, second_nominal - 0.5), second_nominal + 0.5, 0.1)
    )
    for first_s in first_times:
        for turn_s in frange(max(0.1, best_turn["turn_s"] - 0.4), best_turn["turn_s"] + 0.4, 0.1):
            for second_candidate in second_times:
                second_s = first_s if shared_forward_time else second_candidate
                candidate_count += 1
                commands, end_xy, end_yaw = run_candidate(model, data, first_s, turn_s, second_s)
                pos_error = float(np.linalg.norm(end_xy - target_xy))
                yaw_error = abs(((end_yaw - target_yaw + 180.0) % 360.0) - 180.0)
                score = pos_error + 0.02 * yaw_error
                candidate = {
                    "score": score,
                    "commands": commands,
                    "end_xy": end_xy,
                    "end_yaw": end_yaw,
                    "pos_error": pos_error,
                    "yaw_error": yaw_error,
                }
                if best_plan is None or candidate["score"] < best_plan["score"]:
                    best_plan = candidate

    payload = {
        "source": "mujoco",
        "model": str(MODEL_PATH),
        "search": {
            "candidates_evaluated": candidate_count,
            "planner": "Python grid search over MuJoCo rollouts",
            "score": "position_error_m + 0.02 * yaw_error_deg",
            "shared_forward_time": shared_forward_time,
        },
        "goal": {
            "description": "Drive forward, turn right 90 degrees, then drive forward.",
            "first_forward_m": args.first_forward_m,
            "turn_deg": args.turn_deg,
            "second_forward_m": args.second_forward_m,
            "target_xy_m": [round(float(v), 4) for v in target_xy],
            "target_yaw_deg": target_yaw,
        },
        "plan": [
            {"command": command, "seconds": round(seconds, 3)}
            for command, seconds in best_plan["commands"]
        ],
        "sim_predicted_end": {
            "xy_m": [round(float(v), 4) for v in best_plan["end_xy"]],
            "yaw_deg": round(float(best_plan["end_yaw"]), 2),
            "pos_error_m": round(best_plan["pos_error"], 4),
            "yaw_error_deg": round(best_plan["yaw_error"], 2),
        },
    }

    args.out.write_text(json.dumps(payload, indent=2) + "\n")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
