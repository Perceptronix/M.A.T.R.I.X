# ugv_sim — Member 1: UGV + Gazebo Environment + Basic Vehicle

Simulated outdoor UGV in ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8), driven over
`/cmd_vel` with `/odom` feedback. This package covers Member 1 of the
autonomous navigation project only (vehicle, world, drivetrain, sensor
mounts) — no planning/control/perception algorithms are implemented here.

## 1. Requirements

- Ubuntu 24.04
- ROS 2 Jazzy (`/opt/ros/jazzy`)
- Gazebo Harmonic (`gz sim`, version 8.x)
- ROS/Gazebo bridge packages: `ros_gz_sim`, `ros_gz_bridge`
- `xacro`, `robot_state_publisher`
- `colcon`

All of the above are assumed already installed system-wide (`ros-jazzy-desktop`
+ `ros-jazzy-ros-gz`). They are also declared as `exec_depend` in
`package.xml` so `rosdep` can install them on a fresh machine.

## 2. Build

```bash
cd ~/matrix_working
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Launch (single command)

```bash
ros2 launch ugv_sim simulation.launch.py
```

This one command starts, in order:

1. `gz sim` with `worlds/outdoor.world.sdf`, running (`-r`)
2. `robot_state_publisher` (publishes TF from the xacro-processed URDF)
3. `ros_gz_bridge parameter_bridge` (bridges `/cmd_vel`, `/odom`, and the
   sensor topics between ROS 2 and Gazebo Transport)
4. `ros_gz_sim create`, which spawns the `ugv` model from `/robot_description`
   at `(0, 0, 0.5)`

Wait a few seconds after launch for the Gazebo GUI/server and bridge to come
up before publishing commands.

## 4. Topic list

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gazebo |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS |
| `/imu/data` | `sensor_msgs/msg/Imu` | Gazebo → ROS |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | Gazebo → ROS |
| `/camera/left/image_raw` | `sensor_msgs/msg/Image` | Gazebo → ROS |
| `/camera/right/image_raw` | `sensor_msgs/msg/Image` | Gazebo → ROS |
| `/camera/left/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → ROS |
| `/camera/right/camera_info` | `sensor_msgs/msg/CameraInfo` | Gazebo → ROS |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | robot_state_publisher / Gazebo |
| `/robot_description` | `std_msgs/msg/String` | latched, used to spawn the model |
| `/joint_states` | `sensor_msgs/msg/JointState` | published by Gazebo's joint state system |

## 5. Manual movement commands

Forward:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

Backward:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.5}, angular: {z: 0.0}}"
```

Rotate left:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Rotate right:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}"
```

Stop:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```
(Ctrl+C after ~1 second, or let a `timeout 1 ros2 topic pub --rate 10 ...` wrapper end it.)

**Notes:**
- The `gz-sim-diff-drive-system` plugin itself has no command-timeout/deadman
  switch — it applies the last received `Twist` forever. **Fixed in the
  final acceptance test (see section 12):** `ugv_sim` now ships its own
  `cmd_vel_watchdog` node (launched automatically by
  `simulation.launch.py`), which publishes a zero `Twist` to `/cmd_vel`
  whenever the topic has been silent for >0.3 s, so the vehicle fails safe
  to a stop shortly after any command source goes quiet — no explicit stop
  message is required to guarantee the robot halts.
- A bare `ros2 topic pub --once ...` stop can still occasionally be dropped
  by DDS discovery timing (a freshly started CLI process publishing before
  discovery matches it to the bridge's subscriber) — this was reproduced
  directly during acceptance testing (the robot drove ~7.5 m past an
  intended stop point after a 0.5 s-long `--rate 10` stop attempt was
  entirely missed). The watchdog above now bounds the consequence of this
  to ≤0.3 s of continued motion regardless. Prefer `--rate 10` for at least
  0.5–1 s over `--once` for any manually scripted stop, as defense in depth.

## 6. Sensor topics

Sensor mounts are implemented in `urdf/ugv_sensors.xacro` and attached to
`base_link` via fixed joints:

- `lidar_link` — 3D `gpu_lidar` (360° horizontal × 16 vertical channels,
  ±15° vertical FOV, 0.1–30 m range) → `/lidar/points`
- `imu_link` — `imu` sensor → `/imu/data`
- `camera_left_link` / `camera_right_link` — 0.12 m stereo baseline,
  640×480, 60° HFOV cameras → `/camera/left(right)/image_raw` +
  `/camera/left(right)/camera_info`

All sensors are verified live-publishing (IMU orientation data, ~8 Hz lidar
point cloud, ~9–11 Hz camera frames) with the vehicle in the current world.
No perception/processing is implemented against them — that is later
members' work.

## 7. TF frames

```
odom
  └── base_link   (dynamic, ~30 Hz, from gz-sim-diff-drive-system via the bridge's /tf)
        ├── lidar_link          (static)
        ├── imu_link            (static)
        └── camera_left_link / camera_right_link   (static)
```

`robot_state_publisher` publishes the static `base_link → sensor` transforms
(`/tf_static`) from the URDF. The dynamic `odom → base_link` transform is
published by Gazebo's `DiffDrive` plugin (`frame_id: odom`,
`child_frame_id: base_link`) and reaches ROS via an explicit
`/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V` bridge entry in
`launch/simulation.launch.py`.

**Fixed during integration testing:** this `/tf` bridge entry was originally
missing. The DiffDrive plugin was correctly configured with a Gazebo-side
`tf_topic`, but nothing carried it into ROS, so `ros2 topic echo /tf`
returned nothing and `odom → base_link` did not exist in ROS TF at all
(confirmed via `ros2 topic info /tf --verbose` showing only
`robot_state_publisher`, zero live `/tf` traffic, and `view_frames` missing
the `odom` frame). Adding the bridge line fixed it; `view_frames` now shows
`base_link`'s parent as `odom` at ~29.7 Hz. There is only one active
publisher of `odom → base_link` (the bridge) — no duplicate/conflicting TF
sources.

The four wheel joints (`continuous`) are **not** in the TF tree — see Known
limitations.

**Sensor message `frame_id` vs. TF frame names.** SDFormat's URDF→SDF
conversion (used internally when `ros_gz_sim create` spawns the robot)
reduces every fixed-jointed link (`lidar_link`, `imu_link`,
`camera_left_link`, `camera_right_link`) into the single physical link
`base_link`. Because of this, gz-sim's sensor plugins report each sensor's
own physical parent as `ugv/base_link/<sensor_name>` (e.g.
`ugv/base_link/lidar`) — this is what appears in the `header.frame_id` of
`/lidar/points`, `/imu/data`, and both camera topics. `robot_state_publisher`
publishes TF using the *original*, un-reduced URDF link names instead, so
these two naming schemes never matched: a consumer calling
`tf_buffer.lookup_transform('base_link', msg.header.frame_id, ...)` on any
sensor message would fail with a nonexistent-frame error, even though the
sensor topics themselves were publishing live, correct data. **Fixed in the
final acceptance test:** `simulation.launch.py` now also starts four
`tf2_ros static_transform_publisher` nodes that alias each
`ugv/base_link/<sensor_name>` frame to `base_link`, using the same
translation already declared for that sensor's joint in
`ugv_sensors.xacro`. Verified via a live `tf2_ros.Buffer.lookup_transform`
call: both `base_link -> ugv/base_link/lidar` and `base_link -> lidar_link`
(and the equivalent pairs for the IMU and both cameras) now resolve, with
translations matching the URDF joint origins exactly (e.g. lidar
`z=0.480`, camera `x=0.420, y=±0.060, z=0.300`).

## 8. Known limitations

- The four wheel joints are `continuous` but nothing publishes real
  `/joint_states` for them from physics, so wheel rotation is not reflected
  in TF/RViz (it *is* correct in Gazebo's own physics/visuals — this only
  affects RViz-side wheel-spin visualization, not the actual drivetrain or
  odometry).
- `robot_state_publisher` prints one harmless startup warning: `base_link`
  (the URDF root) has an inertia, which KDL doesn't support on a root link.
  This does not affect simulation, physics, or motion.
- No IMU/camera noise models are configured (sensors currently report clean,
  noise-free data) — add Gazebo sensor noise plugins later if the perception
  team needs it.
- The 360° LiDAR sees part of the vehicle's own chassis at short range
  (~0.49 m) off to the side/rear of the sensor — expected for a full-360°
  lidar on a boxy chassis with no self-filtering configured. It does not
  affect forward-facing obstacle detection (verified in section 10).
- **No autonomous navigation, obstacle avoidance, or path planning is
  implemented in this package.** `ugv_sim` contains only the vehicle,
  world, sensors, and drivetrain. `setup.py` declares exactly one
  console_script (`cmd_vel_watchdog`, a vehicle-level safety fail-safe, not
  a planner) and no planner/costmap/avoidance code exists anywhere in the
  workspace (`~/matrix/src` contains only this one package). Autonomous
  obstacle-avoidance and goal-reaching depend on later team members'
  planner/controller nodes consuming `/lidar/points`, `/odom`, etc. — this
  is explicitly out of Member 1's scope.
- A single `ros2 topic pub --once` command can occasionally be dropped by
  DDS discovery timing — see section 5's note. The `cmd_vel_watchdog` node
  bounds the consequence of this to ≤0.3 s of continued motion; still
  prefer `--rate` for scripted commands as defense in depth.
- **Real-time factor is not guaranteed to be 1.0 on this host.** Both
  `gz sim server` and `gz sim gui` were observed pegged near 100% CPU
  during acceptance testing, with instantaneous `real_time_factor` on
  `/world/<world>/stats` fluctuating between ~0.27 and ~1.1. This means a
  fixed wall-clock-duration `/cmd_vel` command does not reliably produce a
  fixed simulated displacement — e.g. a 3 s wall-clock forward command at
  the same commanded velocity was measured to produce anywhere from ~0.26 m
  to ~1.0 m of travel depending on host load at the time. `/odom`'s
  reported instantaneous velocity was confirmed correct and stable
  throughout (matches commanded value exactly while driving, per section
  12) — only the *distance covered per unit of wall-clock time* varies.
  Any timing-sensitive test or future closed-loop controller should key
  off `/odom`/sim time, not wall-clock command durations.

## 9. Obstacle course

`worlds/outdoor.world.sdf` defines a START → GOAL route for later autonomy
testing, plus one dedicated LiDAR test target:

- **`lidar_test_box`** at `(4, 0)` — directly ahead of spawn on the default
  +x heading, isolated from the route below, for straight-line
  approach/retreat distance testing.
- **START** = spawn point `(0, 0)`. **GOAL** = `goal_marker` (visual-only,
  no collision) at `(8, 5)`.
- **`route_obstacle_A`** (rock, r=0.4 m) at `(3, 1.9)` — sits almost exactly
  on the direct START→GOAL line (`y ≈ 0.625x`).
- **`route_obstacle_B`** (box, 0.6 m) at `(5, 4.6)` — offset to one side,
  further along the route.
- **`route_obstacle_C`** (log, cylinder) at `(6.5, 2.8)` — lies across the
  route near the goal approach.
- **`route_obstacle_D`** (box, 0.6 m) at `(7.3, 5.6)` — forces a final
  direction change just before the goal.

The direct straight line from START to GOAL is blocked by `route_obstacle_A`
and passes close to `route_obstacle_C`/`D` — a controller driving straight
at the goal will collide; reaching the goal requires an actual path
deviation.

## 10. LiDAR obstacle detection — measured results

With the UGV facing `+x` (yaw≈0°) toward `lidar_test_box` (near face at
`x=3.7`), forward-cone (±12°) minimum range measured directly from live
`/lidar/points` at each robot position:

| Robot x | Measured forward-cone min range | Geometric expectation |
|---|---|---|
| 0.5664 | 3.1333 m | 3.7 − 0.5664 = 3.1336 m |
| 1.4072 (drove forward) | 2.2914 m | 3.7 − 1.4072 = 2.2928 m |
| 1.9616 (drove forward) | 1.7373 m | 3.7 − 1.9616 = 1.7384 m |
| 1.1132 (drove back) | 2.5865 m | 3.7 − 1.1132 = 2.5868 m |

Measured LiDAR range matches the geometric ground truth to within ~1.5 mm at
every step. Distance measurably **decreases monotonically** on approach
(3.1333 → 2.2914 → 1.7373 m) and **increases** on retreat (→ 2.5865 m) — the
LiDAR is genuinely observing the obstacle, not returning static/fake data.

## 11. Autonomous navigation status

**Not implemented — out of Member 1's scope.** No planner, costmap, or
obstacle-avoidance node exists in this workspace. The vehicle, sensors, and
`/cmd_vel`↔`/odom` interface are fully verified and ready for another
member's planner (e.g. DWA/Hybrid A*) to command `/cmd_vel` using
`/lidar/points` + `/odom`. Claiming autonomous goal-reaching here would be
false — it has not been attempted because the code to attempt it does not
exist yet.

## 12. Acceptance test results (last full integration run)

Full end-to-end verification: clean process kill → `rm -rf build install log`
→ `colcon build --symlink-install` → `ros2 launch ugv_sim simulation.launch.py`
→ scripted `/cmd_vel` publishes via a dedicated rclpy test node → `/odom`
readback (quaternion → yaw). Two real defects were found and fixed during
this run (see sections 5 and 7): the missing `/cmd_vel` deadman timeout
(`cmd_vel_watchdog` node added) and the sensor-message/TF frame_id mismatch
(`static_transform_publisher` aliases added).

Regression results (post-fix, same build):

```
FORWARD    vx held at 0.5000 m/s throughout drive; x increased monotonically; final vx=0.0000
BACKWARD   vx held at -0.5000 m/s throughout drive; x decreased monotonically; final vx=0.0000
ROT LEFT   wz held at 0.5000 rad/s; yaw 29.71° -> 88.96° (increase); final wz=0.0000
ROT RIGHT  wz held at -0.5000 rad/s; yaw 88.96° -> 31.58° (decrease, opposite sense); final wz=0.0000
STOP       after combined vx=0.3/wz=0.3 motion: final vx=0.0000, wz=0.0000 (measured, not assumed)
```

Forward increases x, backward decreases x, left rotation increases yaw,
right rotation decreases yaw (opposite sense), and velocity returns to zero
after every stop. All sensor topics (`/lidar/points`, `/imu/data`, both
cameras + camera_info, `/odom`, `/tf`) were confirmed live-publishing with
real, non-static, non-zero data on this same build, including while the
vehicle was actively moving: all five topics' timestamps were sampled
concurrently during a 4 s drive and showed strictly monotonically
increasing stamps with no freezes; left/right camera frames were confirmed
cryptographically distinct (different MD5 hashes, not duplicated data);
IMU angular velocity tracked a commanded 0.6 rad/s rotation up to a
measured 0.5036 rad/s (realistic physical ramp-up, not an instant echo of
the command) and settled to ~1e-17 rad/s after stop; TF `odom -> base_link`
translation matched the concurrently-read `/odom` position exactly.

**Known host-performance caveat:** `real_time_factor` fluctuated between
~0.27 and ~1.1 during this run (both `gz sim server` and `gz sim gui`
observed at ~100% CPU) — see the real-time-factor bullet in section 8. This
affects only how much *wall-clock* time a given simulated maneuver takes,
not the correctness of any measured relationship (velocity commanded vs.
reported, direction of motion, stop convergence, LiDAR range vs. geometry)
verified above.
