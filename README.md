# M.A.T.R.I.X — Autonomous Off-Road UGV (ROS 2 Jazzy + Gazebo Harmonic)

A colcon workspace for a multi-member autonomous ground-vehicle project:
Gazebo simulation, LIDAR/stereo perception, entropy-gated semantic
segmentation, Hybrid A*/DWA/CBF navigation, and a real-time web dashboard.

## Quickstart (fresh clone)

**Requirements:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (`gz sim` 8.x).

```bash
git clone <this-repo-url> matrix_working
cd matrix_working

# 1. System/ROS dependencies (rosdep reads each package's package.xml)
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 2. Python deps for matrix_semantic (onnxruntime/opencv/etc.)
source /opt/ros/jazzy/setup.bash
python3 -m venv ~/matrix_torch_venv --system-site-packages
source ~/matrix_torch_venv/bin/activate
pip install -r requirements.txt

# 3. Fetch the Gazebo Fuel world (NOT stored in git, see below)
bash scripts/fetch_fuel_world.sh

# 4. Build
colcon build --symlink-install
source install/setup.bash

# 5. Run everything (sim + perception + semantic + navigation + RViz + dashboard)
ros2 launch matrix_demo matrix_demo.launch.py
# dashboard: http://localhost:8088
```

To run subsystems individually instead (e.g. while developing one package),
launch each in its own terminal with `install/setup.bash` sourced:
`ugv_sim simulation.launch.py`, `matrix_perception perception.launch.py`,
`matrix_semantic semantic.launch.py`, `matrix_navigation navigation.launch.py`,
`matrix_dashboard dashboard.launch.py` — see `src/matrix_demo/launch/matrix_demo.launch.py`
for the exact args/timing each one is launched with.

See `docs/FINAL_END_TO_END_SYSTEM_ACCEPTANCE_REPORT.md` for the current,
honestly-reported state of an end-to-end run (what works, what's blocked, and why).

## Why `scripts/fetch_fuel_world.sh`?

`src/ugv_sim/worlds/fuel/rubicon_world/models/Rubicon/` is a **verbatim
copy** of the public Gazebo Fuel model
[`OpenRobotics/models/Rubicon`](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rubicon).
Its mesh alone (`meshes/rubicon.dae`) is ~190MB — over GitHub's 100MB
per-file push limit — so it is **not committed** (see `.gitignore`).
Instead, run `scripts/fetch_fuel_world.sh` once after cloning; it re-downloads
the exact same content via `gz fuel download` and places it where
`ugv_sim`'s launch file (`GZ_SIM_RESOURCE_PATH`) expects it. Verified
byte-identical to the original vendored copy before this was set up.

## Layout

```
matrix_working/
├── src/                     # colcon source packages (ament, one per member/subsystem)
│   ├── ugv_sim/             # Member 1 — vehicle, Gazebo world, URDF/xacro, cmd_vel watchdog
│   ├── matrix_perception/   # Member 2 — C++ perception: ground plane, elevation,
│   │                        #   roughness, step height, clearance, entropy, IAKF fusion,
│   │                        #   traversability costmap, gating supervisor
│   ├── matrix_semantic/     # Member 3 — entropy-gated PIDNet semantic segmentation (ONNX)
│   ├── matrix_navigation/   # Members 4/5 — Hybrid A* planner, DWA, CBF safety filter
│   │                        #   (CBF is the sole writer of /cmd_vel)
│   ├── matrix_dashboard/    # Real-time mission dashboard (stdlib http.server + PIL, :8088)
│   ├── matrix_demo/         # RViz launch/config for demos
│   ├── image_pipeline/      # vendored ROS perception image-processing stack
│   └── image_common/        # vendored ROS image_common support libraries
├── docs/                    # cross-package acceptance/integration reports (all *.md)
├── scripts/
│   └── fetch_fuel_world.sh  # re-downloads the Rubicon Fuel world (see above)
├── requirements.txt         # extra pip deps for matrix_semantic, on top of ROS
└── external/
    └── matrix_upstream/     # vendored upstream reference repo (PIDNet + Perception
                             #   source); has its own nested .git, COLCON_IGNORE'd,
                             #   NOT needed to build/run, NOT committed to this repo
```

Not committed (all regenerable — see `.gitignore`):
`build/`, `install/`, `log/` (colcon build output), `__pycache__/`,
`src/ugv_sim/worlds/fuel/rubicon_world/models/Rubicon/` (fetched, see above),
`external/matrix_upstream/` (vendored reference only).

## Docs index (`docs/`)

- `FINAL_END_TO_END_SYSTEM_ACCEPTANCE_REPORT.md` — latest full-system run report
- `MEMBER2_INTEGRATION_ACCEPTANCE_REPORT.md`, `MEMBER2_MEMBER3_FINAL_INTEGRATION_ACCEPTANCE_REPORT.md`
- `UGV_SIM_*.md` — Member 1 (`ugv_sim`) design/migration/interface reports

Per-package `README.md` files stay inside their own `src/<package>/` folder,
per ROS 2 convention.
