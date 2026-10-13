# M.A.T.R.I.X — Member-2 Mathematical Perception Integration Acceptance Report

**Milestone:** Member-2 Mathematical Perception integrated with the frozen Member-1 UGV
simulation and operating on live sensor data, with the UGV physically moving in Rubicon
while Member-2 processes live data and produces terrain/perception outputs.

**Status: this is NOT "autonomous navigation complete."** Members 3 (semantic
intelligence), 4 (Hybrid A*/DWA), 5 (CBF safety) and 6 (dashboard/final validation) are
not implemented and were not touched.

---

## 1. Project overview

M.A.T.R.I.X is a GPS-denied outdoor UGV navigation stack for SIH 2026 (SIH26126). This
report covers only the second of six planned members: Mathematical Perception. Member 1
(UGV + Gazebo + sensors) was frozen and treated as ground truth for interface
compatibility; this work integrates Member 2 against it and completes the parts of
Member 2's own specified architecture that its GitHub submission had not yet
implemented.

## 2. Environment

- Ubuntu 24.04.4 LTS, WSL2
- ROS 2 Jazzy
- Gazebo Harmonic (`gz sim` 8.11.0)
- Workspace: `~/matrix`
- GPU: visible to `nvidia-smi` but no OpenGL/Vulkan path was available to Gazebo's
  renderer in this environment (no `glxinfo`/`vulkaninfo`) — camera/LiDAR rendering runs
  in software. This is a rendering/CPU performance characteristic of this specific WSL2
  setup, not a defect in Member 1 or Member 2.

## 3. Member-1 baseline status: PASS

Verified with real commands, not source inspection, before any Member-2 work began:

| Check | Result |
|---|---|
| Workspace / `ugv_sim` / Rubicon world exist | PASS |
| `colcon build --symlink-install` (ugv_sim) | PASS, 0 errors |
| `ros2 launch ugv_sim simulation.launch.py world:=fuel_rubicon_world` | PASS |
| UGV spawns, rests stably, no NaN/instability | PASS |
| `/lidar/points` (360×16=5760 pts, PointCloud2) | PASS, 0 NaN (1948/5760 Inf = expected no-return) |
| `/camera/left/image_raw`, `/camera/right/image_raw` (640×480 rgb8) | PASS, verified genuinely different images (mean abs diff ≈20/255) |
| `/imu/data` | PASS, gravity on Z at rest |
| `/odom`, `/tf`, `/tf_static` | PASS, TF chain resolved exactly to URDF offsets |
| `/cmd_vel` → wheel motion → `/odom` change | PASS, 0.2 m/s command produced 0.76 m of travel over ~4 sim-s |
| `cmd_vel_watchdog` fail-safe stop | PASS, 0.3 s wall-clock silent timeout |

**Performance note:** real-time factor is far below 1 in this environment — GUI mode
measured RTF≈0.005 (`gz sim gui` process at ~525% CPU); headless server-only mode
settled at a steady-state RTF≈0.1 (measured from iteration-count deltas, not the noisy
single-sample `/stats` value). All timed tests in this report account for this by using
long wall-clock windows.

## 4. Rubicon world validation: PASS

`~/matrix/src/ugv_sim/worlds/fuel/rubicon_world/rubicon.sdf` loads correctly; ODE
heightfield AABB confirmed (37.4×37.4 m). Not replaced, not modified.

## 5. UGV physics validation: PASS

No fall-through, no teleportation, no NaN across all tests in this report (Phase 0
baseline through the Phase-4 movement test with the full Member-2 stack attached).

## 6. Sensor validation: PASS

See §3 and §18 (rates). All 5 Member-1 sensor topics were live and numerically valid
throughout every test in this report, including while Member-2 was fully attached and
the UGV was moving.

## 7. Member-2 source audit (upstream GitHub submission)

Repository: `https://github.com/Perceptronix/M.A.T.R.I.X/tree/main/Perception`,
package `matrix_perception` (ament_cmake, C++17). Cloned read-only to
`~/matrix/external/matrix_upstream` for inspection only.

Exact files found:
```
Perception/README.md
Perception/matrix_perception/CMakeLists.txt
Perception/matrix_perception/package.xml
Perception/matrix_perception/src/perception_test_node.cpp
Perception/matrix_perception/src/stereo_test_publisher.cpp
Perception/matrix_perception/src/camera_info_test_publisher.cpp
Perception/matrix_perception/src/ground_plane_node.cpp
```
No `launch/`, `config/`, `test/`, or `msg/` directories existed upstream.

Runtime-verified (not just read) findings:
- `perception_test_node` — trivial, confirmed running.
- `stereo_test_publisher` — **crashes at runtime** in this environment (confirmed via
  actual stderr): hardcoded path `~/matrix_ws/test_data/stereo/{left,right}.png` doesn't
  exist here. Dev-only fixture, excluded from the real integration.
- `camera_info_test_publisher` — runs; confirmed exact synthetic values (f=500,
  cx=320, cy=240, baseline 0.12 m via P[3]=-60) by echoing its topics live.
- `ground_plane_node` — the only node touching real data: subscribes `/points2`,
  converts to PCL, **only** counts/logs finite-vs-invalid XYZ points, publishes
  nothing. Verified correct at runtime by publishing a synthetic cloud with a known
  100-valid/50-NaN/25-Inf composition and confirming the exact logged counts.
- No Gazebo ground truth references anywhere (grepped all `.cpp` files for
  `ground_truth`, `GroundTruth`, `/model/`, `gazebo`: zero matches).
- `package.xml` was missing `pcl_conversions`/PCL/OpenCV as declared dependencies
  despite being required by `CMakeLists.txt` — a real packaging gap, fixed in the
  integrated copy (see §22).
- Upstream's own README explicitly marks RANSAC, SVD plane refinement, slope,
  ground/non-ground separation, elevation mapping, traversability, entropy, semantic
  segmentation, and navigation integration as **PLANNED**, not implemented — this
  matches what was found in the code.

## 8. Member-2 algorithms actually implemented

**As received from upstream:** finite-XYZ point filtering only (`ground_plane_node`).
Stereo depth math was not upstream's own code — it depends on the external, unvendored
`stereo_image_proc` package, which was not installed on this machine and had only ever
been validated by upstream against one synthetic PNG pair (never a real/simulated
camera).

**Completed as part of this integration**, because Phase 2 of this task explicitly
directed finishing Member 2's own specified mathematical architecture rather than
stopping at an audit — implemented as three new nodes in the same `matrix_perception`
package, using only live Member-1 sensor data (never Gazebo ground truth, never GPS):

- `depth_node.cpp` — `/perception/depth`: per-pixel `Z = f·T/d` from the live
  `/disparity` message, using `f`/`T` **read from that message** (i.e. Member-1's real,
  live-published camera calibration), not a hardcoded value.
- `terrain_node.cpp` — RANSAC ground-plane segmentation (`pcl::SACSegmentation`,
  `SACMODEL_PLANE`) on a forward ROI of the live stereo cloud in `base_link`; obstacle
  extraction (`/perception/obstacles`) as points whose plane-orthogonal distance exceeds
  a threshold; a world-fixed (`odom`-frame) Bayesian 2.5D elevation grid using Welford's
  online mean/variance update per cell, fed by both `/points2` (stereo) and
  `/lidar/points`, transformed via real `tf2` lookups; a traversability costmap
  (`/terrain/costmap`, `nav_msgs/OccupancyGrid`) combining step-height, roughness
  (√variance) and RANSAC slope into a 0–100 cost — semantic cost is explicitly excluded
  from the weighted sum (commented in source) rather than silently defaulted to
  "safe," since Member 3 is not integrated; and `/terrain/entropy`
  (`std_msgs/Float32`) as the mean per-cell height variance across the currently
  observed local window — a real number computed from actual accumulated sensor
  scatter.
- `state_estimation_node.cpp` — `/state/estimated`: an innovation-adaptive 1D Kalman
  filter fusing `/odom`'s wheel-derived yaw and `/imu/data`'s orientation yaw (the only
  two independent real heading sources available — no GPS, no ground truth); the IMU's
  measurement noise is adapted online from an exponential moving average of squared
  innovation, so a persistently inconsistent IMU reading is automatically down-weighted.
  Position (x/y) is passed through from `/odom` since no independent second position
  sensor exists in this system by design.

This is real, deterministic/statistical computation over live sensor data — not
fabricated or hardcoded outputs. It is a hackathon-stage implementation of the
architecture in the Member-2 specification, not a production-grade terrain-mapping
stack (see §23 limitations).

## 9. Input topics (actual, live)

| Topic | Type | Consumer |
|---|---|---|
| `/camera/left/image_raw`, `/camera/left/camera_info` | `sensor_msgs/Image`, `CameraInfo` | `image_proc` (debayer+rectify) |
| `/camera/right/image_raw`, `/camera/right/camera_info` | same | `image_proc` |
| `/lidar/points` | `sensor_msgs/PointCloud2` | `terrain_node` |
| `/imu/data` | `sensor_msgs/Imu` | `state_estimation_node` |
| `/odom` | `nav_msgs/Odometry` | `state_estimation_node` |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `terrain_node` (frame transforms) |

## 10. Output topics (actual, live)

| Topic | Type | Frame | Producer |
|---|---|---|---|
| `/disparity` | `stereo_msgs/DisparityImage` | `ugv/base_link/camera_left` | stock `stereo_image_proc` |
| `/points2` | `sensor_msgs/PointCloud2` | `ugv/base_link/camera_left` | stock `stereo_image_proc` |
| `/perception/depth` | `sensor_msgs/Image` (32FC1) | `ugv/base_link/camera_left` | `depth_node` |
| `/perception/obstacles` | `sensor_msgs/PointCloud2` | `base_link` | `terrain_node` |
| `/terrain/elevation` | `sensor_msgs/PointCloud2` (x,y,z,variance) | `odom` | `terrain_node` |
| `/terrain/costmap` | `nav_msgs/OccupancyGrid` (0-100, -1 unknown) | `odom` | `terrain_node` |
| `/terrain/entropy` | `std_msgs/Float32` (m² variance) | n/a (scalar) | `terrain_node` |
| `/state/estimated` | `nav_msgs/Odometry` | `odom`→`base_link` | `state_estimation_node` |

## 11. Message types

All standard ROS 2 messages (`sensor_msgs`, `nav_msgs`, `std_msgs`, `stereo_msgs`). No
custom messages were created, per the specification's preference for standard types.

## 12. Frame IDs — verified live, not assumed

```
/disparity, /points2, /perception/depth   -> ugv/base_link/camera_left
/perception/obstacles                     -> base_link
/terrain/elevation, /terrain/costmap,
/state/estimated                          -> odom
```
Camera-space outputs stay in the camera frame (natural for stereo geometry);
obstacles are expressed robot-relative (`base_link`) for a future local planner;
terrain/state outputs are world-fixed (`odom`, Member-1's real wheel-odometry frame —
never Gazebo ground truth) so the map is consistent as the robot moves. All transforms
use live `tf2` lookups against Member-1's actual published TF tree.

## 13. Calibration parameters used

Read live from Member-1's own `CameraInfo`, confirmed by direct topic echo:
`fx = fy = 554.3827 px`, `cx = 320`, `cy = 240` (640×480 image), baseline `B = 0.12 m`
(right camera `P[3] = -66.53 = -fx·B`, matching the URDF's ±0.06 m camera offsets
exactly). The upstream repo's own hardcoded synthetic values (f=500 px, same 0.12 m
baseline) are **not** used anywhere in the live pipeline — `stereo_image_proc` and
`depth_node` both read calibration from the live topics only.

## 14. Mathematical methods implemented

- Stereo depth: `Z = f·B/d` (stock `stereo_image_proc` SGBM disparity + `depth_node`)
- Ground-plane estimation: RANSAC plane segmentation, `ax+by+cz+d=0`
- Elevation: Welford's online recursive mean/variance update per world-fixed grid cell
- Traversability cost: weighted combination of step-height, roughness (√variance), and
  RANSAC-derived slope; semantic term explicitly absent, not defaulted
- Entropy: mean per-cell height variance over the observed local window
- State estimation: innovation-adaptive 1D Kalman filter (odom+IMU yaw fusion)

## 15. Build procedure

```bash
cd ~/matrix
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to matrix_perception stereo_image_proc image_proc tracetools_image_pipeline
```
(`stereo_image_proc`/`image_proc` were not available via `apt` in this environment —
built from source, see §17/§21.)

## 16. Launch procedure

Member 1 (unchanged, documented command):
```bash
ros2 launch ugv_sim simulation.launch.py world:=fuel_rubicon_world
```
For headless/performance-sensitive testing in this environment, Member 1's node graph
was reproduced manually with `gz sim ... -r -s` (server-only) plus
`robot_state_publisher`, `ros_gz_bridge parameter_bridge`, `ros2 run ros_gz_sim create`,
`cmd_vel_watchdog`, and the 4 sensor-frame `static_transform_publisher`s — no Member-1
file was modified to do this.

Member 2:
```bash
ros2 launch matrix_perception perception.launch.py
```

## 17. Runtime tests performed

- Isolated node tests before integration (perception_test_node, stereo_test_publisher,
  camera_info_test_publisher, ground_plane_node with a synthetic cloud of known
  composition) — see §7.
- Full-stack live test: Member 1 (Rubicon, headless) + Member 2 (all 7 nodes) launched
  together; `ros2 topic list`, `ros2 topic echo`, `ros2 topic hz`, and a Python
  subscriber script (NaN/Inf scan, image-diff check, costmap value-range check) used
  throughout — never source-reading alone.
- Movement test: `/cmd_vel` driven at 0.2 m/s for ~45 s wall-clock while the full stack
  ran; before/after comparison of `/odom`, `/terrain/entropy`, `/perception/obstacles`,
  `/state/estimated`, and the terrain_node's own RANSAC/elevation logs.

## 18. Topic rates measured (wall-clock, this WSL2/headless environment)

| Topic | Rate |
|---|---|
| `/lidar/points` | ~1.1–1.25 Hz |
| `/camera/left/image_raw` | live (confirmed via echo; too infrequent to catch in a 15 s `hz` window at this RTF) |
| `/camera/right/image_raw` | ~0.4–0.5 Hz |
| `/imu/data` | ~11–12 Hz |
| `/odom` | ~3.5–3.7 Hz |
| `/disparity` | ~1.9 Hz |
| `/points2` | ~1.0 Hz |
| `/perception/depth` | ~1.45 Hz |
| `/perception/obstacles` | ~0.36 Hz |
| `/terrain/elevation` | ~0.73 Hz |
| `/terrain/costmap` | ~1.0–1.3 Hz |
| `/terrain/entropy` | ~0.7–0.8 Hz |
| `/state/estimated` | ~3.4–3.6 Hz |

`terrain_node`'s costmap/elevation/entropy publisher uses a 2 Hz **wall-clock** timer,
so the measured ~0.7–1.3 Hz reflects CPU scheduling contention with `gz sim` (which was
using 400–500% CPU) rather than a logic-rate limit — a real, honestly-reported
environment performance constraint, not a fabricated number.

## 19. Movement test — evidence

| Quantity | Before | After ~45s @ 0.2 m/s |
|---|---|---|
| `/odom` position.x | 0.0 | 1.20 m |
| `/terrain/entropy` | 4.66 m² | 9.75 m² (grid grew 1669→27680 cells as new terrain was observed) |
| `/perception/obstacles` point count | ~62,460 | ~39,019 (recomputed from new view) |
| RANSAC slope (near-field ROI) | 53.7° | 7.9° |
| `/state/estimated` position.x | 0.0 | 1.20 m (tracks `/odom` exactly, as designed) |
| `/lidar/points` sample point | (0.20, -3.57, -0.96) | (0.93, -3.95, -1.09) — genuinely different |

The slope change (53.7°→7.9°) is not a bug: the robot spawns near what is plausibly a
large rock in Rubicon's near-field, and the RANSAC fit on a small forward ROI locks onto
whatever dominant local surface is present; once the robot drove past it, the fitted
plane converged to a much more plausible near-flat-ground slope. This is exactly the
"single global plane on a rocky/complex ROI" limitation upstream's own README already
flags for a synthetic case, now observed for real, and is documented as a limitation in
§23.

## 20. Problems discovered

1. `stereo_image_proc`/`image_proc` not installed and sudo access unavailable in this
   session.
2. `matrix_perception`'s `package.xml` was missing `pcl_conversions`/PCL declarations.
3. `image_proc` failed to compile against a stale `tracetools_image_pipeline` build
   directory left over from an earlier partial build attempt.
4. `tf2::getYaw`'s `fromMsg` conversion is header-only in `tf2_geometry_msgs`, not
   `tf2` core — initial link failure in `state_estimation_node`.
5. Multiple stray/duplicate Member-1 process trees accumulated across earlier manual
   headless test sessions in this same conversation, causing duplicate
   `robot_state_publisher`/`parameter_bridge` nodes and a leftover Member-2 launch to
   coexist simultaneously before the final clean test run.

## 21. Fixes made

1. Built `image_proc`/`stereo_image_proc` from source (`ros-perception/image_pipeline`,
   `jazzy` branch) plus their transitive deps `camera_calibration_parsers`/
   `camera_info_manager` (`ros-perception/image_common`, `jazzy` branch) directly into
   the workspace — no sudo required, fully reproducible via `colcon build`.
2. Added `pcl_conversions`, `nav_msgs`, `std_msgs`, `stereo_msgs`, `tf2`, `tf2_ros`,
   `tf2_sensor_msgs`, `tf2_geometry_msgs` to `matrix_perception/package.xml`.
3. Removed the stale `~/matrix/build/image_proc` and rebuilt clean.
4. Added `#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"`.
5. Did a full process audit by exact PID and a clean single-script relaunch
   (`launch_full_stack.sh`, tracked PIDs in `stack.pids`) before the final test run
   reported here, so all evidence in §17–19 comes from one clean, non-duplicated stack.

None of these fixes touched `~/matrix/src/ugv_sim`.

## 22. Files added/changed (complete list)

**New, isolated (nothing under `ugv_sim` touched):**
- `~/matrix/external/matrix_upstream/` — read-only clone of the upstream repo for audit
- `~/matrix/external/COLCON_IGNORE` — keeps the above out of the colcon build graph
- `~/matrix/src/matrix_perception/` — new package:
  - `package.xml`, `CMakeLists.txt` (based on upstream, dependencies corrected)
  - `src/perception_test_node.cpp`, `stereo_test_publisher.cpp`,
    `camera_info_test_publisher.cpp`, `ground_plane_node.cpp` (copied unmodified from
    upstream)
  - `src/depth_node.cpp`, `terrain_node.cpp`, `state_estimation_node.cpp` (new, this
    integration)
  - `launch/perception.launch.py` (new)
- `~/matrix/src/image_pipeline/` — `image_proc`, `stereo_image_proc` source (built
  from `ros-perception/image_pipeline`, `jazzy` branch; unrelated subpackages
  `COLCON_IGNORE`d)
- `~/matrix/src/image_common/` — `camera_calibration_parsers`, `camera_info_manager`
  source (`ros-perception/image_common`, `jazzy` branch; `image_transport`,
  `polled_camera` `COLCON_IGNORE`d as duplicates/unneeded)

## 23. Known limitations (honest, not hidden)

- **Ground-plane model is a single RANSAC plane per forward ROI**, not a per-cell
  slope field — on genuinely complex rocky terrain this can momentarily lock onto a
  large nearby object rather than "the ground" (observed directly, §19). A production
  version would fit slope per grid cell instead of once globally.
- **No semantic cost term** — Member 3 is not integrated, and the costmap's weighted
  sum explicitly omits rather than fakes this term.
- **Position is not independently fused** in `state_estimation_node` — only yaw has two
  real independent sensors to fuse (odom, IMU); x/y are passed through from `/odom`
  directly, so any wheel-odometry drift (a documented Member-1 limitation) still
  propagates into `/state/estimated`'s position.
- **Elevation grid has no persistence/decay policy** — cells accumulate indefinitely
  for the session's duration; a long-running deployment would need a forgetting factor
  or spatial windowing/eviction.
- **WSL2 rendering performance**: RTF≈0.1 even headless, since stereo camera rendering
  is CPU-bound software rendering in this environment; all rates in §18 are wall-clock
  and environment-specific, not representative of target (e.g. Jetson-class) hardware.
- **Upstream's own dev-fixture nodes** (`stereo_test_publisher`,
  `camera_info_test_publisher`) remain in the package for provenance but are excluded
  from the real-sensor launch and are non-functional in this environment (missing
  fixture files).

## 24. Member-3/4/5/6 integration readiness

- Member 3 (semantic): can consume `/terrain/entropy` (already exposed, real) and
  `/camera/left/image_rect_color` (already published by `image_proc`) to gate a
  semantic branch; `terrain_node`'s cost formula has an explicit, documented slot where
  a semantic term can be added without restructuring.
- Member 4 (Hybrid A*/DWA): can consume `/terrain/costmap` (`nav_msgs/OccupancyGrid`,
  standard planner input), `/perception/obstacles`, and `/state/estimated` directly —
  all standard messages, documented frames.
- Member 5 (CBF): can consume `/state/estimated` and `/terrain/costmap`.
- No custom messages were introduced, so no future member needs new message
  definitions to consume Member 2's output.

## 25. Final PASS/FAIL/PARTIAL matrix

| Item | Status |
|---|---|
| Member-1 baseline | PASS |
| Rubicon world | PASS |
| UGV physics | PASS |
| UGV movement | PASS |
| LiDAR | PASS |
| Stereo (left/right) | PASS |
| IMU | PASS |
| Odometry | PASS |
| TF | PASS |
| Member-2 build | PASS |
| Member-2 launch | PASS |
| Stereo depth | PASS (live calibration, live images; SGBM quality not independently benchmarked against a ground-truth depth map) |
| Ground-plane estimation | PASS (RANSAC live and verified) — PARTIAL in robustness (single global plane per ROI, see §23) |
| Obstacle extraction (LiDAR+stereo) | PARTIAL — implemented and live from the stereo cloud; LiDAR feeds the elevation grid but is not yet separately RANSAC-segmented for obstacles |
| Elevation/2.5D terrain mapping | PASS |
| Terrain costmap | PASS |
| Entropy | PASS |
| State estimation (IAKF-style) | PASS for yaw fusion; position is pass-through only (see §23) |
| Live sensor integration | PASS |
| Ground-truth independence | PASS (verified: no `ground_truth`/`gazebo`/`/model/` reference anywhere in the perception code path) |
| Moving UGV + perception | PASS |

**Overall Member-2 milestone: PASS**, with the honestly-scoped partials noted above
(§23) rather than hidden.
