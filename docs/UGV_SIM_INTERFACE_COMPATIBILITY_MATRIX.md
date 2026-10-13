# INTERFACE COMPATIBILITY MATRIX

Status column: **LIVE** = verified publishing/consuming real data this audit. **PROTOTYPE** = implemented in Member 4's existing (out-of-scope-to-modify) prototype, not yet the final architecture. **PLANNED** = not implemented anywhere yet, Members 2-6 future work.

| Component | Inputs | Outputs | Message Type | Frame | Rate | Dependencies | Downstream User | Status |
|---|---|---|---|---|---|---|---|---|
| Vehicle / DiffDrive | `/cmd_vel` | `/odom`, `/tf` (`odom→base_link`) | `geometry_msgs/Twist` in; `nav_msgs/Odometry`, `tf2_msgs/TFMessage` out | `odom` | cmd: as published; odom/tf: 30 Hz | `gz-sim-diff-drive-system` | Every planner/controller member | **LIVE** |
| `cmd_vel_watchdog` (C++) | `/cmd_vel` | `/cmd_vel` (zero Twist on 0.3 s silence) | `geometry_msgs/Twist` | — | 20 Hz check | rclcpp, geometry_msgs | Vehicle (fail-safe layer) | **LIVE** |
| LiDAR | — | `/lidar/points` | `sensor_msgs/PointCloud2` | `ugv/base_link/lidar` (alias: `base_link`) | 10 Hz configured, ~8-10 Hz observed | `gz-sim-sensors-system` (gpu_lidar) | Member 2 (terrain/obstacles), Member 4 prototype | **LIVE** |
| Stereo cameras | — | `/camera/{left,right}/image_raw`, `/camera/{left,right}/camera_info` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` | `ugv/base_link/camera_{left,right}` (alias: `base_link`) | 15 Hz configured | `gz-sim-sensors-system` (camera) | Member 2 (stereo depth), Member 3 (semantic) | **LIVE** |
| IMU | — | `/imu/data` | `sensor_msgs/Imu` | `ugv/base_link/imu_sensor` (alias: `base_link`) | 100 Hz configured | `gz-sim-imu-system` | Member 2 (state estimation) | **LIVE** |
| TF | — | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `odom` root | dynamic ~30 Hz, static once | `robot_state_publisher`, DiffDrive, 4× `static_transform_publisher` | All members (frame lookups) | **LIVE** |
| Local occupancy costmap | `/lidar/points`, `/odom` | *(internal to `navigation_node.py`, not published as a standalone topic)* | — | `odom` | rebuilt on decay cycle | Member 4 prototype's `costmap.py` | Member 4 prototype's own planner only | **PROTOTYPE** |
| Terrain roughness | `/lidar/points` (near-ground band), `/odom` | *(internal, same as above)* | — | `odom` | continuous | Member 4 prototype's `costmap.py` | Same as above | **PROTOTYPE** |
| `/terrain/costmap` | Member 2 | Member 2 | `nav_msgs/OccupancyGrid` (recommended — standard, no custom message needed) | `odom` | TBD by Member 2 | Member 2's own perception pipeline | Member 4 (final), Member 6 (viz) | **PLANNED** |
| `/terrain/elevation` | Member 2 | Member 2 | TBD — likely `sensor_msgs/PointCloud2` or a grid message; standard types should be evaluated first | `odom` | TBD | Member 2 | Member 4, Member 6 | **PLANNED** |
| `/terrain/entropy` | Member 2 | Member 2 | `std_msgs/Float32` (per-cell array or scalar summary — TBD by Member 2's design) | `odom` | TBD | Member 2 | Member 3 (entropy-gated activation), Member 4 | **PLANNED** |
| `/perception/obstacles` | Member 2 | Member 2 | TBD — standard types (`vision_msgs/Detection3DArray` or similar) should be evaluated before a custom message | `odom` | TBD | Member 2 | Member 4 | **PLANNED** |
| `/perception/depth` | Member 2 (stereo) | Member 2 | `sensor_msgs/Image` or `PointCloud2` | camera frame or `odom` | TBD | Member 2 | Member 4, Member 3 | **PLANNED** |
| `/state/estimated` | Member 2 (IAKF or similar) | Member 2 | `nav_msgs/Odometry` (reuse, don't invent a new type) | `odom` | TBD | Member 2 | Member 4 (replaces raw `/odom` consumption) | **PLANNED** |
| `/semantic/class_map`, `/semantic/confidence` | Member 3 | Member 3 | TBD | camera frame | TBD, entropy-gated (not continuous) | Member 3, depends on Member 2's entropy output | Member 4 (optional cost term) | **PLANNED** |
| Global path | `/odom` (target: `/state/estimated`), costmap (target: `/terrain/costmap`) | `/planner/global_path` | `nav_msgs/Path` | `odom` | on replan (prototype: rate-limited ≥4.5 s between replans) | Member 4's Hybrid A* | Member 6 (viz), Member 5/vehicle indirectly | **PROTOTYPE** (final version pending Members 2/3 inputs) |
| Local trajectory / command | Global path, costmap | `/planner/local_trajectory` (viz), `/cmd_vel` today → **`/planner/proposed_cmd_vel`** once Member 5 exists | `nav_msgs/Path`, `geometry_msgs/Twist` | `odom` | 10 Hz control loop | Member 4's DWA | Member 5 (target architecture) or Vehicle (prototype, current) | **PROTOTYPE**; topic rename required for Member 5 integration |
| `/safety/cmd_vel` | `/planner/proposed_cmd_vel`, obstacle/footprint/uncertainty data | Member 5 | `geometry_msgs/Twist` | — | control loop rate | Member 5's CBF filter | `/cmd_vel` → Vehicle | **PLANNED** |
| RViz / metrics / scenarios | All of the above topics | — | — | — | — | Member 6 | Human operator | **PLANNED** |

## Notes on message-type decisions

Per this audit's own instruction ("if an interface needs a custom message, do NOT immediately create it — first determine whether standard ROS 2 messages are sufficient"): every **PLANNED** row above proposes a standard message type as the first option. Only `/terrain/elevation`, `/perception/obstacles`, and `/semantic/class_map` are genuinely open questions that should be settled when Members 2/3 are actually designed, not guessed here.
