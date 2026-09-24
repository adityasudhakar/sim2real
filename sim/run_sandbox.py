#!/usr/bin/env python3
import argparse
import math
import time
from pathlib import Path

try:
    import mujoco
    import numpy as np
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Missing dependency. Run: python3 -m venv .venv && "
        ". .venv/bin/activate && pip install mujoco numpy"
    ) from exc


ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT / "toy_cleaner_sandbox.xml"


SCENARIOS = {
    "clear_forward": {
        "rover_xy": (0.0, -0.65),
        "yaw_deg": 0.0,
        "toy_xy": (0.0, 0.55),
        "description": "Open path toward a block.",
    },
    "nose_corner": {
        "rover_xy": (-1.33, 1.33),
        "yaw_deg": -45.0,
        "toy_xy": (0.0, 0.55),
        "description": "Nose pointed into a room corner.",
    },
    "front_wall": {
        "rover_xy": (0.0, 1.33),
        "yaw_deg": 0.0,
        "toy_xy": (0.0, 0.55),
        "description": "Front bumper facing a wall with no switch signal.",
    },
    "side_post": {
        "rover_xy": (-0.27, 0.10),
        "yaw_deg": 8.0,
        "toy_xy": (0.0, 0.75),
        "description": "Side wedged near a chair-leg-like post.",
    },
    "low_block_drag": {
        "rover_xy": (0.0, 0.28),
        "yaw_deg": 0.0,
        "toy_xy": (0.0, 0.80),
        "description": "Front edge close to a low block.",
    },
}


COMMANDS = {
    # Calibrated from the real cardboard rover over a 49 in / 1.245 m run:
    # 3.274s, 3.480s, 3.618s => about 0.361 m/s average forward speed.
    # Right turn calibration over 90 deg: 1.216s, 0.867s, 0.887s, 1.088s
    # => about 1.0145s average for a 90 deg right turn.
    "forward": {"left": -12.7, "right": -12.7},
    "backward": {"left": 12.7, "right": 12.7},
    "turn_left": {"left": 16.0, "right": -16.0},
    "turn_right": {"left": -16.0, "right": 16.0},
    "stop": {"left": 0.0, "right": 0.0},
}


SCRIPTED_PLANS = {
    "probe_forward": [("forward", 2.0)],
    "probe_backward": [("backward", 2.0)],
    "probe_turn_left": [("turn_left", 2.0)],
    "probe_turn_right": [("turn_right", 2.0)],
    "escape_left": [("backward", 1.4), ("turn_left", 1.2), ("forward", 1.0)],
    "escape_right": [("backward", 1.4), ("turn_right", 1.2), ("forward", 1.0)],
}


def yaw_quat(yaw_rad):
    return np.array([math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)])


def set_freejoint_pose(model, data, joint_name, xy, yaw_deg, z):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        return
    qpos_adr = model.jnt_qposadr[joint_id]
    data.qpos[qpos_adr : qpos_adr + 3] = [xy[0], xy[1], z]
    data.qpos[qpos_adr + 3 : qpos_adr + 7] = yaw_quat(math.radians(yaw_deg))


def body_xy(model, data, body_name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return data.xpos[body_id, :2].copy()


def reset_scenario(model, data, name):
    scenario = SCENARIOS[name]
    mujoco.mj_resetData(model, data)
    set_freejoint_pose(model, data, "rover_free", scenario["rover_xy"], scenario["yaw_deg"], 0.0)
    set_freejoint_pose(model, data, "toy_free", scenario["toy_xy"], 0.0, 0.08)
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)


def rover_yaw(model, data):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rover")
    mat = data.xmat[body_id].reshape(3, 3)
    return math.atan2(mat[1, 0], mat[0, 0])


def apply_motion_command(_model, data, command):
    spec = COMMANDS[command]
    data.ctrl[0] = spec["left"]
    data.ctrl[1] = spec["right"]


def step_command(model, data, command, seconds):
    steps = int(seconds / model.opt.timestep)
    for _ in range(steps):
        apply_motion_command(model, data, command)
        mujoco.mj_step(model, data)
    data.ctrl[:] = 0.0
    for _ in range(int(0.15 / model.opt.timestep)):
        mujoco.mj_step(model, data)


def run_plan(model, data, scenario_name, plan_name):
    reset_scenario(model, data, scenario_name)
    start = body_xy(model, data, "rover")
    for command, seconds in SCRIPTED_PLANS[plan_name]:
        step_command(model, data, command, seconds)
    end = body_xy(model, data, "rover")
    displacement = float(np.linalg.norm(end - start))
    return {
        "scenario": scenario_name,
        "plan": plan_name,
        "start_xy": start.round(3).tolist(),
        "end_xy": end.round(3).tolist(),
        "displacement_m": round(displacement, 3),
        "low_progress": displacement < 0.08,
    }


def run_headless(args):
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    for plan_name in args.plan:
        result = run_plan(model, data, args.scenario, plan_name)
        print(result)


def run_viewer(args):
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    reset_scenario(model, data, args.scenario)
    plan = SCRIPTED_PLANS[args.plan[0]]
    command_index = 0
    command_until = data.time + plan[0][1]
    command = plan[0][0]

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.distance = args.viewer_distance
        viewer.cam.azimuth = args.viewer_azimuth
        viewer.cam.elevation = args.viewer_elevation
        viewer.cam.lookat[:] = args.viewer_lookat
        if args.viewer_start_delay > 0:
            end_delay = time.monotonic() + args.viewer_start_delay
            while viewer.is_running() and time.monotonic() < end_delay:
                mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(0.03)
        while viewer.is_running():
            if command_index < len(plan) and data.time >= command_until:
                command_index += 1
                if command_index < len(plan):
                    command, seconds = plan[command_index]
                    command_until = data.time + seconds
                else:
                    command = "stop"
            apply_motion_command(model, data, command)
            mujoco.mj_step(model, data)
            viewer.sync()
            if args.viewer_sleep > 0:
                time.sleep(args.viewer_sleep)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), default="clear_forward")
    parser.add_argument(
        "--plan",
        choices=sorted(SCRIPTED_PLANS),
        nargs="+",
        default=["probe_forward", "escape_left", "escape_right"],
    )
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--viewer-sleep", type=float, default=0.0)
    parser.add_argument("--viewer-start-delay", type=float, default=0.0)
    parser.add_argument("--viewer-distance", type=float, default=3.0)
    parser.add_argument("--viewer-azimuth", type=float, default=-130.0)
    parser.add_argument("--viewer-elevation", type=float, default=-35.0)
    parser.add_argument("--viewer-lookat", type=float, nargs=3, default=[0.0, 0.0, 0.0])
    args = parser.parse_args()

    print(f"scenario: {args.scenario} - {SCENARIOS[args.scenario]['description']}")
    if args.viewer:
        run_viewer(args)
    else:
        run_headless(args)


if __name__ == "__main__":
    main()
