# MEMBER 1 ACCEPTANCE REPORT

Package: `ugv_sim` (`~/matrix/src/ugv_sim`)
Platform: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8.15.0)
World: Gazebo Fuel `Rubicon` (OpenRobotics), local copy at `worlds/fuel/rubicon_world/`
Exact launch command:

```bash
cd ~/matrix
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ugv_sim simulation.launch.py world:=fuel_rubicon_world
```

Every PASS below is backed by a command actually run and output actually read during this audit (build logs, live topic echoes, `gz model -p` ground truth, a dedicated rclpy/C++ regression harness) — not by a node existing or a topic being declared.

---

## 1. Vehicle architecture — PASS

Defined across `urdf/ugv.urdf.xacro` (chassis + 4 wheels), `urdf/ugv_gazebo.xacro` (DiffDrive plugin), `urdf/ugv_sensors.xacro` (LiDAR/IMU/stereo cameras). Current values (verified by direct file read this audit):

| Property | Value |
|---|---|
| Chassis (box) | 0.80 × 0.60 × 0.20 m |
| Chassis mount height | 0.40 m above wheel-axle plane (belly clearance 0.30 m) |
| Chassis mass | 18.0 kg |
| Chassis inertia (ixx/iyy/izz) | 1.03 / 1.54 / 2.06 |
| Wheel radius / width | 0.16 m / 0.10 m |
| Wheel mass (each) | 3.0 kg |
| Wheel positions | (±0.28, ±0.35, 0.16) m |
| Wheel joints | 4× `continuous`, axis `(0,1,0)` |
| Chassis friction (mu1/mu2) | 1.0 / 1.0 |
| Wheel friction (mu1/mu2) | 2.5 / 2.5 |
| Total mass | 30.0 kg |

**Note on these values:** the chassis-height, friction, and mass parameters were changed from their original values (0.25 m mount / 1.5 wheel friction / 55 kg total) during terrain-physics work earlier in this session, each change measured and documented in the source file comments. They are Member-1-scope vehicle parameters (physical configuration), not navigation-algorithm tuning, and are kept because they measurably improved terrain climbing without introducing instability (see §9).

## 2. Gazebo architecture — PASS

`gz-sim-diff-drive-system` plugin (`urdf/ugv_gazebo.xacro`): `wheel_separation=0.70`, `wheel_radius=0.16`, topic `/cmd_vel`, `odom_topic=/odom`, `tf_topic=/tf`, `frame_id=odom`, `child_frame_id=base_link`, `odom_publish_frequency=30`. No PosePublisher or other competing plugin present (a prior attempt to add one broke DiffDrive entirely and was reverted; confirmed absent by direct read this audit).

## 3. World integration — PASS

`worlds/fuel/rubicon_world/rubicon.sdf`: explicit system plugins (Physics, Sensors/ogre2, SceneBroadcaster, UserCommands, Imu, Contact), 1 ms physics step, `<include><uri>model://Rubicon</uri></include>` pointing at the locally-copied model (`worlds/fuel/rubicon_world/models/Rubicon/`, ~342 MB, not fetched from network at launch). Fresh launch this audit: no errors, no missing-mesh/texture warnings, UGV spawned at `(0, 0, 0.5)` and settled level.

## 4. UGV physics — PASS

Tested this audit (`gz model -m ugv -p` ground truth + `/odom`, dedicated harness):

- **Stand still:** roll=pitch=yaw=0°, stable indefinitely.
- **Forward:** commanded `vx=0.4`; both true position and `/odom` advanced in +x; pitch responded to real terrain (up to ~30° climbing rock near spawn) — genuine terrain interaction, not flat/frictionless motion.
- **Backward:** commanded `vx=-0.4`; both true position and `/odom` advanced in -x; pitch reduced as the vehicle backed off the slope.
- **Rotate left/right:** `/odom` yaw changed in the commanded direction each time (opposite signs, confirmed).
- **Stop:** `/odom` linear/angular velocity returns to exactly 0.0 after a stop command (confirmed via live topic echo, not a single noisy sample).
- **Terrain traversal / slope / dent / stop-on-uneven-terrain:** vehicle physically responds to real Rubicon rock geometry — pitch and (occasionally) roll go nonzero as required by the terrain, no fall-through, no tunneling, wheel contact maintained, settles to a stable rest pose on uneven ground. This matches extensive earlier testing this session (structured multi-scenario driving tests recorded pitch up to ~42° near a boulder formation at spawn, always recoverable, never unstable at the current physics configuration).
- Terrain was **not** flattened, collisions were **not** disabled, the robot was **not** teleported, and no ground-truth pose was used as a substitute for real motion at any point in this audit.

## 5. Sensor audit — PASS

All verified live this audit with a dedicated rclpy harness (not topic-existence only):

| Topic | Type | Result |
|---|---|---|
| `/lidar/points` | `sensor_msgs/PointCloud2` | 5760 points/scan (360×16, matches sensor config), 5241-5248 finite returns, range 0.49-29.8 m, frame `ugv/base_link/lidar` |
| `/camera/left/image_raw`, `/camera/right/image_raw` | `sensor_msgs/Image` | 640×480 `rgb8`; left/right MD5 hashes confirmed **distinct** (real stereo, not duplicated) |
| `/imu/data` | `sensor_msgs/Imu` | live; `linear_acceleration.z ≈ 9.8` (gravity, correct sign/magnitude), orientation ≈ identity at rest |
| `/odom` | `nav_msgs/Odometry` | live position/orientation/velocity, tracks commanded motion (§4) |
| `/tf`, `/tf_static` | — | see §6 |

## 6. TF tree — PASS

`odom → base_link` (dynamic, DiffDrive via bridge) confirmed live and resolvable via `tf2_ros.Buffer.lookup_transform`. Four sensor-frame aliases (`base_link → ugv/base_link/{lidar,imu_sensor,camera_left,camera_right}`, published by `static_transform_publisher` nodes in the launch file) all resolve with translations exactly matching the current URDF joint origins (lidar z=0.630, imu z=0.410, cameras x=0.420/y=±0.060/z=0.450) — these were re-verified to match after this session's chassis-clearance change, which also shifted every sensor mount height by +0.15 m.

These aliases exist because SDFormat's URDF→SDF fixed-joint reduction merges every sensor link into the single physical `base_link`, so gz-sim's sensor plugins report `frame_id=ugv/base_link/<sensor>` while `robot_state_publisher` (working from the un-reduced URDF) publishes TF under the original link names. This is a real, structural SDFormat behavior, not a Member 1 defect — the aliasing is the correct fix and is verified working.

## 7. Odometry — PASS (with a documented, explained limitation)

`/odom` tracks commanded motion correctly in direction and general magnitude (§4). It is **pure wheel-velocity dead reckoning with no ground-truth correction** — this is inherent to the `gz-sim-diff-drive-system` plugin, not a bug, and is the correct, honest odometry source for anything consuming this package (no ground-truth substitute is provided or should be). During aggressive wheel-slip maneuvers (climbing/recovering on steep rock), `/odom` has been measured to drift from true pose by over 0.35 m within ~15 s — documented in `ugv_sim/navigation/navigation_node.py`'s comments as a known, real, and actionable characteristic for any consumer building a map or planner from it.

## 8. Terrain interaction — PASS

Covered in §4. Additional evidence from this session's earlier structured terrain testing (7-scenario A-G driving test suite): real, repeatable pitch response up to ~30-42° depending on physics configuration tested, roll response up to several degrees on genuinely uneven ground, always recoverable, never a fall-through/tunnel/NaN event across dozens of test runs at the current (post-fix) physics configuration.

## 9. Obstacle sensing — PASS

`/lidar/points` returns real geometry from the actual Rubicon model: rocks, a stump, terrain undulation, and the ranger-station building are all within LiDAR range and were confirmed present in the model's own collision data (`model.sdf` contains 241 individual collision shapes: `Rock01_Collider_*`, `rock02_collider_*`, `rockpile_collider_*`, `StumpCollider*`, plus the main terrain heightmap and a building `boundary` collider). LiDAR min/max range this audit (0.49-29.8 m) is consistent with a genuinely cluttered, non-trivial scene. No planner exists in Member 1's scope to consume this — this section verifies only that the sensor foundation contains what a later planner will need.

## 10. Build status — PASS

```bash
cd ~/matrix && source /opt/ros/jazzy/setup.bash
rm -rf build install log && colcon build --symlink-install
```

Clean build from a fully wiped `build/install/log`, zero errors, zero warnings surfaced by colcon, reproducible (run twice this session with identical success). Package build type is now `ament_cmake` (previously `ament_python`; see §12 and `CPP_MIGRATION_REPORT.md`).

## 11. C++ migration status — PASS (scoped)

`cmd_vel_watchdog` (the Member 1 `/cmd_vel` deadman safety node) migrated from Python to C++ (`rclcpp`). Verified as a genuine compiled binary (`file` reports ELF 64-bit executable), runs correctly under `ros2 run ugv_sim cmd_vel_watchdog`, and its core safety behavior was re-verified post-migration with a dedicated timing test: velocity held at the commanded 0.3 m/s through the 0.3 s timeout window, dropped to ~0 immediately after (measured `v` samples: `0.300` at t+0.2s, `3.4e-10` at t+0.4s). See `CPP_MIGRATION_REPORT.md` for full scope rationale and the file-by-file classification.

Member 4's navigation stack (`ugv_sim/navigation/*.py`) and the legacy `goal_navigator.py` were deliberately **not** touched — explicitly out of Member 1's scope per this audit's instructions.

## 12. Regression results (post-C++-migration) — PASS

Full clean rebuild + fresh launch + the same sensor/movement/terrain test harness as §4-5, rerun after the migration:

- Build: clean, zero errors.
- Launch: no errors, UGV spawned and settled cleanly.
- `cmd_vel_watchdog` (now C++) confirmed running automatically via the **unchanged** launch file (no launch-file edits were needed — same `package='ugv_sim', executable='cmd_vel_watchdog'` Node() call resolves to the new C++ binary).
- All 5 sensor topics: PASS, same as §5.
- Forward/backward/rotate-left/rotate-right/stop: PASS, same directional/zero-convergence behavior as pre-migration.
- Terrain traversal: PASS, real pitch/roll response, stable settle, no fall-through.
- `navigation_node` and `goal_navigator` (untouched Python, Member 4/legacy) still launch correctly via `ros2 run` through new wrapper scripts (`scripts/navigation_node`, `scripts/goal_navigator`) that replace the setup.py console_scripts mechanism ament_cmake doesn't provide — verified reaching the exact same code path as before (both correctly imported their package modules and reached the interactive keypress prompt).

## 13. Known limitations

- **`/odom` drift under wheel-slip** (§7) — inherent to dead-reckoning, not a defect, but a real constraint any consumer must design around (Member 4's `navigation_node.py` already documents and partially mitigates this via costmap cell decay).
- **Real-time factor is not guaranteed to be 1.0** on this host. A fresh headless physics-only run this audit measured RTF ≈ 0.42 (42%); the same world with the GUI attached measured RTF as low as 0.002-0.006 (0.2-0.6%) late in this session, after many hours of continuous relaunches and screenshot automation degraded the WSLg rendering pipeline (one `gz sim gui` process was observed consuming 734% CPU). This is a host/WSLg environment characteristic, not a Member 1 code defect — confirmed by isolating physics-only performance from GUI-attached performance (same technique used earlier in this project for the flat-world course). Any timing-sensitive test should key off `/odom`/sim time or use `--headless-rendering`, not wall-clock command durations.
- **`goal_navigator.py`** (legacy, pre-dates Member 4's formal navigation architecture) is currently **broken**: it subscribes to `/ground_truth_pose`, a topic that does not exist in the current `config/bridge.yaml` (removed when an earlier PosePublisher-plugin experiment was reverted after it broke DiffDrive). This file is not part of Member 1's declared responsibilities and was left as-is per this audit's scope rules — flagged here for visibility, not fixed.
- **`README.md` is significantly stale**: it states "no planner/costmap/avoidance code exists anywhere in the workspace" and describes only the flat `outdoor.world.sdf` course — both were true when written but predate the entire Member 4 `navigation/` package and the Rubicon world integration. Worth a documentation pass, not a code fix.
- Wheel joints are `continuous` with no real `/joint_states` publisher, so wheel spin is not reflected in RViz/TF (cosmetic only — physics and odometry are unaffected).
- No IMU/camera sensor noise models configured (clean data only); add later if a perception team needs it.

## 14. Fixes made during this audit

No new Member 1 **bugs** were found and fixed during this specific audit pass — the one substantive Member 1 defect (inadequate ground clearance for Rubicon terrain, causing the vehicle to high-center immediately at spawn) was found and fixed earlier in this session (chassis mount height raised, sensor mounts shifted to match) and is verified still correct and in place (§1, §6). This audit's own testing (§4, §12) found the vehicle and sensor foundation functioning correctly against the current build.
