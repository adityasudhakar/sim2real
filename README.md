# Cardboard Rover Sim2Real

Small MuJoCo-to-real demo for a cardboard two-wheel rover.

The demo calibrates two primitive motions from the real rover:

- straight-line speed over a 49 inch / 1.245 m lane
- right-turn timing over a 90 degree turn

MuJoCo then searches over a short L-path plan:

```text
forward 0.6 m
turn right 90 deg
forward 0.6 m
```

The selected plan is written to `sim_l_path_plan.json`, and the physical rover
executes that same JSON with `execute_plan.py`.

## What This Proves

This is a modest sim2real demo:

```text
real rover calibration -> MuJoCo model constants
MuJoCo rollout/search -> JSON plan
real rover executes JSON plan
```

It demonstrates primitive-calibrated open-loop plan transfer. It does not claim
full robot autonomy, RL policy transfer, camera navigation, or robust obstacle
handling.

## Files

- `sim/toy_cleaner_sandbox.xml` - clean MuJoCo room and rover model
- `sim/run_sandbox.py` - calibrated MuJoCo command primitives
- `sim/make_l_path_plan.py` - MuJoCo grid-search planner
- `sim/render_plan_video.py` - renders the simulated JSON plan to MP4
- `execute_plan.py` - runs the JSON plan on the Raspberry Pi rover
- `time_distance_run.py` - real straight-line calibration helper
- `time_turn_run.py` - real 90-degree turn calibration helper
- `make_side_by_side_video.py` - combines sim and real videos
- `sim_l_path_plan.json` - current generated plan
- `sim_l_path_plan.mp4` - current MuJoCo render

## Sim Setup

```bash
python3.12 -m venv .venv312
. .venv312/bin/activate
pip install -r sim/requirements.txt
```

Generate the current L-path plan:

```bash
python sim/make_l_path_plan.py --out sim_l_path_plan.json
```

Render the MuJoCo side:

```bash
python sim/render_plan_video.py --plan sim_l_path_plan.json --out sim_l_path_plan.mp4
```

## Real Rover

Copy these files to the Pi:

```bash
scp drive.py execute_plan.py sim_l_path_plan.json pi@192.168.4.72:/home/pi/sim2real/
```

Run the sim-generated plan:

```bash
cd /home/pi/sim2real
./execute_plan.py sim_l_path_plan.json
```

## Side-by-Side Video

After filming the real rover executing the plan:

```bash
python make_side_by_side_video.py --real /path/to/real_rover.mov --out rover_sim_real.mp4
```
