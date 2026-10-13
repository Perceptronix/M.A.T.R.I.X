# FUTURE MEMBER INTEGRATION ANALYSIS

Members 2-6 are **not implemented** in this pass. This document analyzes how each should connect to the completed Member 1 foundation, based on what Member 1 actually provides today (verified in `MEMBER_1_ACCEPTANCE_REPORT.md`) and, where relevant, what Member 4's existing prototype (`ugv_sim/navigation/`) already demonstrates.

---

## Member 2 — Mathematical Perception + Terrain Engine

**What it needs from Member 1 (all verified present and live):** `/camera/left/image_raw` + `/camera/right/image_raw` (640×480 `rgb8`, 0.12 m stereo baseline, confirmed genuinely distinct streams), `/lidar/points` (360×16, 5760 pts/scan, real Rubicon geometry), `/imu/data`, `/odom`, `/tf`. All message types, frame conventions, and rates are standard ROS 2 — no custom message needed for input.

**Integration path:** Member 2 subscribes to the above and should publish to `/terrain/costmap` (`nav_msgs/OccupancyGrid`), `/terrain/elevation`, `/terrain/entropy` (`std_msgs/Float32` or a per-cell array), `/perception/obstacles`, `/perception/depth`. All of these are new topics Member 1 does not currently produce — no collision with anything Member 1 owns.

**One real overlap to resolve:** Member 4's existing prototype (`ugv_sim/navigation/costmap.py`) already implements a simplified version of exactly this — a world-frame occupancy grid *and* a sensor-derived terrain-roughness signal (Welford running variance of near-ground LiDAR returns per cell), built directly from `/lidar/points` + `/odom`, with cell decay to counter `/odom` drift smear. This is not Member 2's final design, but it is a working, tested reference for the terrain-roughness half of Member 2's job. When Member 2 is built, either replace this prototype's costmap outright (publishing the standard `/terrain/costmap` instead of keeping it internal to `navigation_node.py`) or keep Member 4's version as a fallback until Member 2 is validated — do not run both writing to the same conceptual state simultaneously.

**Frame/coordinate compatibility:** Member 1's world-frame convention is `odom` (from DiffDrive), not a fixed `map` frame — there is no localization/SLAM in this project. Member 2's elevation grid and costmap should be built and published in `odom`, consistent with what Member 4 already assumes, unless Member 2 introduces its own localization (out of Member 1's scope to decide).

## Member 3 — Lightweight Semantic Intelligence

**What it needs:** camera data (as above) plus Member 2's terrain/entropy output — Member 3 has no direct dependency on raw Member 1 topics beyond the cameras.

**Integration path:** publishes `/semantic/class_map`, `/semantic/confidence`. These are new topics; standard `sensor_msgs/Image` (for a class-map-as-image) or a custom lightweight message may be needed depending on class encoding — **defer that decision to Member 3's own design**; per this audit's own instruction, don't invent a custom message before confirming standard types are insufficient.

**Entropy-gated activation** implies a dependency on Member 2 being live first (Member 3 needs `/terrain/entropy` or equivalent to decide when to run inference) — this makes Member 3 dependent on Member 2, not on Member 1 directly, beyond the shared camera feed.

**Computational note (see risk P1 in `FINAL_ARCHITECTURE_READINESS_REPORT.md`):** this project's actual host performance was measured this session to be highly variable (headless physics-only RTF ≈ 0.42; GUI-attached RTF as low as 0.002-0.006 after extended use). Any onboard inference (ONNX Runtime, even a lightweight model) adds real CPU/GPU load on the same host already straining to render Gazebo — this should be budgeted and measured early, not assumed free.

## Member 4 — Hybrid A* + DWA

**Current state: a working prototype already exists** (`ugv_sim/navigation/`), built and tested during this project's earlier phases, explicitly out of scope for this Member 1 audit to modify further. It already:

- Subscribes to `/odom` (not ground truth — matches the project's own hard requirement) and `/lidar/points`.
- Publishes `/cmd_vel`, plus `/planner/global_path` and `/planner/local_trajectory` (`nav_msgs/Path`) for visualization.
- Implements Hybrid A* with (x,y,θ) state, fixed-arc motion primitives, footprint-based collision checking, and a terrain-cost term.
- Implements DWA with velocity-space sampling, collision/clearance/progress/heading/oscillation/reverse-penalty scoring, and a terrain-cost term.
- Implements a bounded recovery state machine (rotate-in-place and an alternating forward/reverse "rock" maneuver, discovered necessary when constant-velocity driving proved unable to clear a tight rock gap that the same vehicle *could* clear with pulsed motion) that always terminates (never loops forever) and reports `FAILED` honestly when recovery is exhausted.

**Integration path once Members 2/3 exist:** Member 4 should switch its obstacle/terrain input from its own internal `costmap.py` to Member 2's published `/terrain/costmap` + `/perception/obstacles`, and optionally weight `/semantic/class_map` into its terrain-cost term. The existing cost-combination structure (`total_cost = obstacle_cost + terrain_cost + path_length_cost + heading_cost + curvature_cost + clearance_cost`) is already the right shape to accept an additional semantic term without restructuring.

**Expected inputs (target architecture):** `/state/estimated` (replacing raw `/odom` once Member 2/an estimator exists), `/terrain/costmap`, `/perception/obstacles`, semantic info when available, goal. **Expected outputs:** `/planner/global_path`, `/planner/proposed_cmd_vel` (renamed from the prototype's direct `/cmd_vel` publish, to make room for Member 5's safety filter — see below).

**One required change when Member 5 is introduced:** Member 4 currently publishes directly to `/cmd_vel`. For Member 5 to sit between planner and vehicle, Member 4 must be repointed to publish `/planner/proposed_cmd_vel` instead — a one-line topic-name change in `navigation_node.py`, not a redesign.

## Member 5 — CBF Safety + Final Motion Control

**Integration path:** subscribes `/planner/proposed_cmd_vel` (from Member 4, after the rename above) plus obstacle distance/footprint/uncertainty/terrain constraints (from Member 2), publishes `/safety/cmd_vel`, which then becomes the actual `/cmd_vel` Member 1's vehicle consumes.

**Direct compatibility with Member 1:** trivial — Member 1's vehicle interface is exactly "consume `geometry_msgs/Twist` on `/cmd_vel`," and it does not care whether that Twist came from a planner directly or through a safety filter. The existing `cmd_vel_watchdog` (now C++, verified in `MEMBER_1_ACCEPTANCE_REPORT.md`) sits downstream of whatever publishes final `/cmd_vel` and needs no awareness of Member 5 at all — it will fail-safe-stop the vehicle regardless of which upstream node goes silent.

**Explicit caution (per this audit's own instruction):** do not claim formal safety guarantees for a CBF filter without mathematical and experimental validation specific to this vehicle's actual dynamics and footprint (verified in §1 of the acceptance report) — a CBF derived for a different footprint/mass/turning-radius model is not automatically valid here.

## Member 6 — Integration + Visualization + Testing

**Integration path:** RViz config, scenario/experiment management, metrics collection, integration testing across the full stack. Has no direct code dependency on Member 1 beyond consuming the same topics every other member does (for visualization) — its main dependency is on the *launch* architecture.

**Concrete implication for Member 1's launch file:** `launch/simulation.launch.py` currently launches exactly the Member 1 stack (Gazebo, bridge, spawn, watchdog, TF aliases) plus accepts a `world` argument. Member 6's "final launch" should almost certainly **compose** this file (via `IncludeLaunchDescription`) rather than replace it, adding Members 2-5's nodes and an RViz node alongside it — preserving Member 1's own launch file as the stable base, per this audit's instruction not to rewrite working components unnecessarily.
