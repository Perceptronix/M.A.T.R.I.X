# M.A.T.R.I.X — Perception

## SIH26126 — Vision Based Autonomous Navigation for Unmanned Ground Vehicle for Outdoor Environment

This directory contains the **Perception subsystem** of M.A.T.R.I.X, being developed for **Smart India Hackathon 2026 — SIH26126**, focused on vision-based autonomous navigation of an Unmanned Ground Vehicle (UGV) in outdoor environments.

The perception subsystem converts raw sensor observations into structured geometric, terrain, uncertainty, and traversability information that can later be consumed by localization, path planning, dynamic control, and safety filtering.

The development follows a **mathematics-first, deterministic perception architecture**, with lightweight semantic perception introduced when geometric reasoning alone is insufficient.

> **Current implementation status:** ROS 2 Jazzy, stereo geometry, metric depth, 3D point-cloud generation, PCL processing, RANSAC + SVD ground-plane estimation, slope, Bayesian elevation, roughness, clearance, step-height estimation, unified traversability costmap, geometric entropy gating, and a compact IAKF fusion implementation have been implemented and controlled-test validated.

> **Important:** Phase 3 validation is based on controlled synthetic/test fixtures. It does not yet constitute final physical-UGV or outdoor-field validation.

---

# 1. Project Overview

The M.A.T.R.I.X perception subsystem transforms raw sensor measurements into structured information about the surrounding environment.

The current implemented perception pipeline is:

    Stereo Camera / Test Point Cloud
              |
              v
       Stereo Geometry
              |
              v
        Metric Depth
              |
              v
         3D Point Cloud
              |
              v
         PointCloud2
              |
              v
      PCL Conversion
              |
              v
      Finite XYZ Filtering
              |
              v
       Ground Plane
       RANSAC + SVD
              |
       +------+------+
       |             |
       v             v
     Slope      Ground/Obstacle
       |             |
       +------+------+
              |
              v
       Bayesian Elevation
              |
       +------+------+------+
       |      |      |      |
       v      v      v      v
   Roughness Clearance Step  Variance
                       Height
       |      |      |      |
       +------+------+------+
              |
              v
    Traversability Costmap
              |
              v
       Geometric Entropy
              |
              v
       Entropy Gate
              |
       +------+------+
       |             |
       v             v
   Math-only      Semantic
                    Branch
              
Parallel state estimation:
    
    Measurements
         |
         v
       IAKF
         |
         v
    /state/estimated

The complete M.A.T.R.I.X architecture will additionally integrate:

- Stereo camera
- LiDAR
- IMU
- Wheel encoders
- RTK-GNSS
- Multi-source sensor fusion
- Elevation mapping
- Traversability estimation
- Lightweight semantic segmentation
- Visual localization
- Hybrid A* planning
- DWA/local dynamic control
- CBF-QP / MR-CBF safety filtering

Only components explicitly marked as implemented and validated in this README should be considered completed.

---

# 2. Development Philosophy

The perception system is developed incrementally.

Each stage follows:

    Implement
       |
       v
     Build
       |
       v
      Run
       |
       v
    Validate
       |
       v
     Measure
       |
       v
   Document
       |
       v
    Integrate

This prevents the project from becoming a large unverified perception pipeline.

Every major mathematical component is tested independently before being connected to downstream processing.

The project uses the following status definitions:

| Status | Meaning |
|---|---|
| IMPLEMENTED | Code exists and can be executed |
| VALIDATED | Output has been numerically or behaviorally verified |
| IN PROGRESS | Implementation is actively being developed |
| PLANNED | Part of the architecture but not implemented yet |
| DEVELOPMENT FIXTURE | Controlled synthetic/test data used for validation |
| FUNCTIONAL VALIDATION | Spatial/behavioral operation verified, but not absolute physical accuracy |

---

# 3. Development Environment

Current development environment:

- Host OS: Windows 11
- Linux environment: Ubuntu 24.04 LTS
- Virtualization: WSL2
- ROS distribution: ROS 2 Jazzy
- Build system: colcon
- Primary implementation language: C++17
- Computer vision: OpenCV
- Point-cloud processing: Point Cloud Library (PCL)
- ROS stereo processing: `stereo_image_proc`
- Linear algebra: Eigen

Workspace:

    ~/matrix_ws

Main perception package:

    matrix_perception

GitHub working copy:

    ~/matrix_github

Repository perception directory:

    Perception/

Package directory:

    Perception/matrix_perception/

---

# 4. Phase 1 — ROS 2 Perception Workspace Setup

## Status

**COMPLETE**

Phase 1 established the ROS 2 perception development environment and verified that a custom M.A.T.R.I.X perception package could be created, compiled, installed, and executed.

## 4.1 ROS 2 Package

The main package is:

    matrix_perception

The package uses:

    ament_cmake

The package currently includes dependencies required by the implemented perception pipeline, including:

- rclcpp
- sensor_msgs
- nav_msgs
- std_msgs
- geometry_msgs
- cv_bridge
- image_transport
- pcl_conversions
- OpenCV
- PCL
- Eigen

## 4.2 Initial ROS 2 Test Node

The initial perception validation node is:

    perception_test_node.cpp

Its purpose was to verify:

- ROS 2 package creation
- C++ compilation
- executable installation
- ROS 2 node initialization
- runtime execution
- ROS 2 logging
- workspace configuration

The package was successfully built using:

    colcon build --packages-select matrix_perception --symlink-install

The node was successfully executed using:

    ros2 run matrix_perception perception_test_node

## 4.3 Phase 1 Result

Validation:

- ROS 2 Jazzy environment — PASS
- ROS workspace — PASS
- `matrix_perception` package — PASS
- `ament_cmake` build — PASS
- C++ perception node — PASS
- ROS 2 runtime execution — PASS

**Phase 1: COMPLETE**

---

# 5. Phase 2 — Stereo Geometry Foundation

## Status

**COMPLETE — CONTROLLED SYNTHETIC VALIDATION**

Phase 2 established the complete stereo geometry chain from stereo images to metric 3D point-cloud data.

Pipeline:

    Left Image
         |
         v
    Right Image
         |
         v
    stereo_image_proc
         |
         v
        SGBM
         |
         v
     Disparity
         |
         v
     Metric Depth
         |
         v
     Point Cloud

The purpose of this phase was to validate the stereo geometry chain before using 3D data for terrain processing.

---

# 6. Phase 2.1 — Stereo Image Processing

The ROS 2 package:

    stereo_image_proc

was installed and verified.

The stereo processing pipeline was configured with:

- Stereo algorithm: SGBM
- Approximate synchronization: disabled
- Exact message synchronization

Test launch command:

    ros2 launch stereo_image_proc stereo_image_proc.launch.py \
      left_namespace:=left \
      right_namespace:=right \
      stereo_algorithm:=1 \
      approximate_sync:=false

The SGBM stereo pipeline successfully generated disparity from the controlled stereo input.

---

# 7. Phase 2.2 — Controlled Synthetic Stereo Fixture

A controlled synthetic stereo fixture was created to validate the stereo geometry numerically.

Fixture properties:

    Resolution:
    640 x 480 pixels

    Known disparity:
    20 pixels

Stereo parameters:

    f = 500 pixels
    B = 0.12 m

Stereo depth relationship:

    Z = fB / d

For:

    f = 500
    B = 0.12 m
    d = 20 pixels

the expected depth is:

    Z = 3.0 m

This known reference allows the complete stereo geometry chain to be numerically checked.

---

# 8. Phase 2.3 — Stereo Test Publisher

The test publisher:

    stereo_test_publisher.cpp

publishes:

    /left/image_raw
    /right/image_raw

and camera information:

    /left/camera_info
    /right/camera_info

All four messages use the same timestamp so that exact synchronization can operate correctly.

This resolved the initial synchronization issue where independently timestamped image and CameraInfo messages could not be paired reliably.

---

# 9. Phase 2.4 — Stereo Camera Calibration Fixture

The controlled stereo fixture uses:

    K =
    [ 500   0   320 ]
    [   0 500   240 ]
    [   0   0     1 ]

The right camera projection matrix contains:

    P[3] = -60

because:

    -fB = -(500 x 0.12)
        = -60

The controlled fixture assumes:

- Zero distortion
- Identity rotation
- Known stereo baseline

This calibration is for mathematical pipeline validation rather than final physical-camera calibration.

---

# 10. Phase 2.5 — Disparity Validation

Measured results:

    Resolution:
    640 x 480

    Total pixels:
    307200

    Valid disparity pixels:
    161572

    Valid percentage:
    52.60 %

    Expected disparity:
    20.000 px

    Measured mean disparity:
    20.001 px

    Mean absolute disparity error:
    0.003 px

The measured disparity agrees with the known synthetic fixture.

**Result: PASS**

---

# 11. Phase 2.6 — Depth and Point-Cloud Validation

Expected depth:

    Z = 3.000 m

Measured point-cloud results:

    Resolution:
    640 x 480

    Point step:
    32 bytes

    Valid XYZ points:
    161572

    Mean Z:
    3.000 m

    Median Z:
    3.000 m

    Z range:
    approximately 2.972 - 3.038 m

    Mean absolute depth error:
    0.000 m

The measured depth agrees with the known synthetic fixture.

**Result: PASS**

---

# 12. Phase 2.7 — Development Performance Observation

During development under WSL2 using CPU processing:

    Disparity processing:
    approximately 2.8 - 3 Hz

    Point-cloud generation:
    approximately 0.6 Hz

These are development-environment observations only.

They are not final M.A.T.R.I.X performance targets.

The final architecture targets NVIDIA Jetson-class hardware with appropriate optimization and GPU acceleration.

---

# 13. Phase 2 Final Result

Validated chain:

    Stereo Images
          |
          v
      CameraInfo
          |
          v
    Exact Synchronization
          |
          v
         SGBM
          |
          v
      Disparity
          |
          v
     Metric Depth
          |
          v
      Point Cloud

Validation:

- Stereo processing — PASS
- SGBM — PASS
- CameraInfo — PASS
- Exact timestamp synchronization — PASS
- Disparity generation — PASS
- Disparity numerical validation — PASS
- Metric depth — PASS
- Point-cloud generation — PASS
- Point-cloud numerical validation — PASS

**Phase 2: COMPLETE**

### Phase 2 Limitation

The validation uses a synthetic stereo fixture.

It therefore validates the stereo geometry implementation, not final physical-camera outdoor performance.

---

# 14. Phase 3 — Ground Plane, Terrain Geometry & Traversability

## Status

**COMPLETE — CONTROLLED MATHEMATICAL / SYNTHETIC VALIDATION**

Phase 3 expanded the perception pipeline from basic 3D geometry into structured terrain understanding.

The implemented chain is:

    /points2
       |
       v
    PointCloud2
       |
       v
    PCL Conversion
       |
       v
    Finite XYZ Filtering
       |
       v
    ROI Selection
       |
       v
    RANSAC Plane
       |
       v
    SVD Refinement
       |
       v
    Surface Normal
       |
       v
    Slope
       |
       +--------------------+
       |                    |
       v                    v
    Ground Points      Non-Ground Points
       |                    |
       v                    v
    Elevation           Obstacles
       |
       +---------+----------+----------+
       |         |          |          |
       v         v          v          v
    Variance  Roughness  Step Height Clearance
       |         |          |          |
       +---------+----------+----------+
                         |
                         v
               Traversability Cost
                         |
                         v
                  Costmap / 0-100
                         |
                         v
                  Entropy / Gate

A parallel state-estimation path is:

    Measurement
         |
         v
       IAKF
         |
         v
    /state/estimated

---

# 15. Phase 3.1 — Ground Plane Estimation

The main terrain geometry node is:

    ground_plane_node.cpp

Input:

    /points2

Message:

    sensor_msgs/msg/PointCloud2

The point cloud is converted to:

    pcl::PointCloud<pcl::PointXYZ>

using PCL and `pcl_conversions`.

The processing sequence is:

    PointCloud2
        |
        v
    PCL PointCloud
        |
        v
    Valid XYZ Filtering
        |
        v
    ROI
        |
        v
    RANSAC
        |
        v
    SVD Refinement

---

# 16. Phase 3.2 — Finite XYZ Filtering

Each point is checked for finite:

    X
    Y
    Z

coordinates.

A point is valid only when:

    isfinite(x)
    isfinite(y)
    isfinite(z)

This prevents invalid stereo depth values from entering the terrain geometry algorithms.

---

# 17. Phase 3.3 — RANSAC Ground Plane

RANSAC plane estimation is implemented using PCL sample-consensus plane segmentation.

Plane equation:

    ax + by + cz + d = 0

The RANSAC stage identifies the dominant plane in the selected terrain region while rejecting non-plane points and outliers.

The controlled validation used a known inclined plane.

Validation result:

    Estimated plane normal:
    [-0.173648, 0.000000, 0.984808]

    Normal magnitude:
    1.000000

    RANSAC inliers:
    5100

The resulting plane corresponds to a known 10-degree inclination.

**RANSAC plane estimation: PASS**

---

# 18. Phase 3.4 — SVD Plane Refinement

After RANSAC identifies the plane inliers, the plane is refined using the covariance structure of the inlier point distribution.

For centered points:

    q_i = p_i - p_bar

the covariance matrix is constructed from the centered points.

The surface normal is obtained from the eigenvector corresponding to the smallest covariance eigenvalue.

For the controlled fixture:

    SVD refined normal:
    [-0.173648, 0.000000, 0.984808]

    Eigenvalues:
    approximately
    [0,
     0.357445,
     1.333200]

The smallest-eigenvalue eigenvector correctly represents the plane normal.

**SVD refinement: PASS**

---

# 19. Phase 3.5 — Surface Slope

Slope is computed from the refined ground-plane normal.

Using:

    slope = acos(|n_z|)

the controlled fixture produced:

    Expected slope:
    10.000000 degrees

    Estimated slope:
    9.999999 degrees

    Mean error:
    0.000001 degrees

**Slope estimation: PASS**

The current implementation publishes:

    /terrain/slope

as a `std_msgs/msg/Float32MultiArray`.

The current grid representation uses the estimated plane-wide slope value across the local terrain grid.

This should not be interpreted as a final per-cell local slope estimator.

---

# 20. Phase 3.6 — Ground / Non-Ground Classification

The refined plane is used to classify points according to their distance from the estimated ground plane.

Controlled result:

    Total ROI points:
    5438

    Ground:
    5100

    Non-ground:
    338

    Ground distance threshold:
    0.05 m

The minimum non-ground distance was:

    approximately 0.492 m

This provides the geometric obstacle representation used by downstream terrain processing.

The classified non-ground point cloud is published as:

    /perception/obstacles

using:

    sensor_msgs/msg/PointCloud2

---

# 21. Phase 3.7 — Bayesian Elevation Mapping

A local 2.5D elevation grid was implemented.

Grid:

    Width:
    60 cells

    Height:
    60 cells

    Resolution:
    0.10 m

    Spatial extent:
    x,y = [-3, 3] m

Each cell maintains:

- Elevation mean
- Elevation variance
- Observation count
- Initialization state

The Bayesian update is based on:

    K = P_prior / (P_prior + R)

    z_post =
        z_prior + K(z_measurement - z_prior)

    P_post =
        (1-K)P_prior

The development fixture uses a fixed measurement variance of:

    R = 0.01 m²

Validation demonstrated deterministic posterior updates and decreasing variance as repeated observations accumulate.

Example variance sequence:

    0.010000000
    0.001666667
    0.001428571
    0.000833333
    ...
    0.000185185
    0.000172414 m²

The elevation layer is published as:

    /terrain/elevation

and the uncertainty layer is represented internally/published for downstream processing.

**Bayesian elevation mapping: PASS**

---

# 22. Phase 3.8 — Roughness Estimation

A roughness layer was implemented using local elevation statistics.

For each grid cell:

    mean(z) = (1/N) sum(z_i)

    variance =
        (1/N) sum((z_i - mean(z))²)

    roughness = sqrt(variance)

The resulting roughness value therefore represents local vertical surface variation.

Output:

    /terrain/roughness

Message:

    std_msgs/msg/Float32MultiArray

Grid:

    60 x 60

    resolution:
    0.10 m

Controlled validation:

    Valid cells:
    800

    Minimum roughness:
    0.049690351 m

    Maximum roughness:
    0.049999952 m

    Mean roughness:
    0.049922552 m

**Roughness estimation: PASS**

---

# 23. Phase 3.9 — Clearance Estimation

Obstacle clearance was implemented using the classified non-ground point cloud.

Input:

    /perception/obstacles

Output:

    /terrain/clearance

Grid:

    60 x 60

    resolution:
    0.10 m

Maximum configured clearance:

    6.0 m

The implementation estimates the nearest obstacle distance in the local XY terrain grid.

Runtime validation:

    Obstacles received:
    338

    Valid obstacle points:
    338

    Occupied cells:
    72

    Maximum clearance:
    6.00 m

**Clearance estimation: FUNCTIONAL PASS**

The current test validates the operation and spatial representation of clearance. It is not a final physical obstacle-distance accuracy benchmark.

---

# 24. Phase 3.10 — Step-Height Estimation

Step height is computed from neighboring elevation cells.

For each cell:

    h_step(i,j) =
        max(
            |z_i - z_left|,
            |z_i - z_right|,
            |z_i - z_up|,
            |z_i - z_down|
        )

The output is:

    /terrain/step_height

Grid:

    60 x 60

    resolution:
    0.10 m

Controlled validation:

    Valid cells:
    800

    Computed cells:
    800

    Maximum estimated step:
    0.319043 m

    Mean estimated step:
    0.033127 m

Maximum cell:

    ix = 54
    iy = 26

World location:

    x = 2.45 m
    y = -0.35 m

The maximum value occurs immediately adjacent to the known obstacle boundary.

**Step-height estimation: FUNCTIONAL / SPATIAL PASS**

The test validates spatial detection of an elevation discontinuity.

It does not claim absolute recovery of the fixture's full 0.50 m obstacle height because cell aggregation and Bayesian elevation representation affect the measured discontinuity.

---

# 25. Phase 3.11 — Unified Traversability Costmap

A unified terrain costmap was implemented using:

- Slope
- Roughness
- Step height
- Clearance
- Elevation variance

Inputs:

    /terrain/slope
    /terrain/roughness
    /terrain/step_height
    /terrain/clearance
    /terrain/elevation_variance

Output:

    /terrain/costmap

Message:

    nav_msgs/msg/OccupancyGrid

Grid:

    60 x 60

    Resolution:
    0.10 m

    Origin:
    (-3, -3)

The current development weights are:

    slope:
    0.25

    roughness:
    0.20

    step:
    0.25

    clearance:
    0.20

    variance:
    0.10

The normalized cost is:

    C =
      w_slope C_slope
      + w_roughness C_roughness
      + w_step C_step
      + w_clearance C_clearance
      + w_variance C_variance

with the final value clamped to:

    0 <= C <= 1

and represented in the ROS OccupancyGrid as:

    0 - 100

with:

    -1 = unknown

Controlled validation:

    Valid cells:
    800

    Unknown cells:
    2800

    Cost range:
    11 - 73

    Mean:
    24.104

The output range was successfully validated.

**Traversability costmap: PASS**

### Important limitation

The current implementation uses development weights and normalization thresholds.

These are engineering baselines, not final experimentally optimized SIH deployment parameters.

The architecture also defines a slip-related augmentation:

    Delta C_slip =
        kappa_slip ||v||
        |sin(theta_heading - theta_slope)|
        S²

but this slip augmentation is **not implemented in the current node**.

---

# 26. Phase 3.12 — Geometric Entropy

A geometric entropy estimator was implemented to provide an uncertainty/ambiguity signal for the semantic supervisor.

Input:

    /terrain/elevation_variance

Output:

    /terrain/entropy

Message:

    std_msgs/msg/Float32

The current prototype uses localized elevation-variance distributions.

For histogram probabilities:

    p_i

Shannon entropy is:

    H = -sum(p_i log(p_i))

The entropy is normalized using:

    H_normalized =
        H / log(N_bins)

so that:

    0 <= H_normalized <= 1

The implementation uses configurable:

    local_width
    local_height
    bin_count

Controlled tests:

    Uniform variance fixture:
    H = 0

    Mixed variance fixture:
    H = 0.602060

The mixed-variance case successfully produces higher uncertainty than the uniform case.

**Geometric entropy estimator: PASS**

---

# 27. Phase 3.13 — Entropy-Gated Supervisor

A gating supervisor was implemented:

    gating_supervisor_node.cpp

Input:

    /terrain/entropy

Outputs:

    /perception/gate_decision
    /perception/semantic_trigger

The current development threshold is:

    H_threshold = 0.30

Decision rule:

    H < 0.30
        |
        v
    MATH_ONLY

    H >= 0.30
        |
        v
    SEMANTIC_REQUIRED

Controlled validation:

    H = 0
        -> MATH_ONLY

    H = 0.602060
        -> SEMANTIC_REQUIRED

This demonstrates the intended architecture:

    Low geometric ambiguity
             |
             v
      Deterministic path

    High geometric ambiguity
             |
             v
       Semantic branch

**Entropy gate: PASS**

### Important limitation

The architecture-level entropy formulation also includes a visual contrast term:

    H_surface(x,y)
      =
      -integral p(z)log p(z) dz
      +
      beta_vis(1 - Contrast(I))

The current standalone prototype does **not yet implement the camera-contrast component**.

Therefore, this README describes the current implementation as a **geometric entropy prototype**, not the final combined visual-geometric entropy formulation.

---

# 28. Phase 3.14 — Innovation-Adaptive Kalman Filter

A compact Innovation-Adaptive Kalman Filter was implemented:

    iakf_fusion_node.cpp

The current validation state is:

    x = [x, y, v]^T

with constant-velocity prediction:

    x_k^- = F x_(k-1)

where:

    F =
    [1  0  dt]
    [0  1   0]
    [0  0   1]

The measurement model is:

    z = Hx + noise

with:

    H = I

The innovation is:

    nu_k =
        z_k - H x_k^-

A sliding innovation covariance is computed:

    C_y =
        (1/W) sum(nu nu^T)

The measurement covariance is adapted from the observed innovation statistics.

The filter publishes:

    /state/estimated

using:

    nav_msgs/msg/Odometry

Controlled validation demonstrated:

- State convergence
- Adaptive measurement-noise increase under noisy measurements
- Recovery when measurements become stable
- Finite output covariance

Example adaptive measurement covariance increase:

    X:
    0.05000 -> 0.11091

    Y:
    0.05000 -> 0.10768

    V:
    0.05000 -> 0.05988

After stable measurements were restored:

    R -> [0.05000, 0.05000, 0.05000]

Example stable state:

    x  = approximately 1.0328 m
    y  = approximately 2.0000 m
    vx = approximately 0.4988 m/s

**IAKF controlled validation: PASS**

### Important limitation

This is currently a compact standalone IAKF validation model.

It is **not yet the complete production multi-sensor state estimator** combining:

- IMU
- Wheel encoders
- RTK-GNSS
- Stereo/LiDAR-derived state

The full multi-source fusion architecture remains an integration task.

---

# 29. Phase 3 Validation Summary

The current Phase 3 validation results are:

| Component | Result | Validation Type |
|---|---|---|
| PointCloud2 → PCL | PASS | Runtime |
| Finite XYZ filtering | PASS | Numerical |
| RANSAC plane | PASS | Controlled fixture |
| SVD refinement | PASS | Numerical |
| Surface normal | PASS | Numerical |
| Slope | PASS | Numerical |
| Ground/non-ground classification | PASS | Controlled fixture |
| Bayesian elevation | PASS | Numerical |
| Elevation variance | PASS | Numerical |
| Roughness | PASS | Numerical |
| Clearance | FUNCTIONAL PASS | Spatial/runtime |
| Step height | FUNCTIONAL PASS | Spatial/runtime |
| Traversability costmap | PASS | Numerical/runtime |
| Geometric entropy | PASS | Controlled fixture |
| Entropy gate | PASS | Behavioral |
| IAKF | PASS | Controlled convergence/adaptation |
| Clean ROS 2 build | PASS | Build |

---

# 30. Current ROS Interfaces

The current perception implementation provides the following interfaces.

## Inputs

    /points2

    sensor_msgs/msg/PointCloud2

---

## Terrain Outputs

    /perception/obstacles

    sensor_msgs/msg/PointCloud2

    /terrain/elevation

    Terrain elevation grid

    /terrain/roughness

    std_msgs/msg/Float32MultiArray

    /terrain/step_height

    std_msgs/msg/Float32MultiArray

    /terrain/clearance

    std_msgs/msg/Float32MultiArray

    /terrain/slope

    std_msgs/msg/Float32MultiArray

    /terrain/costmap

    nav_msgs/msg/OccupancyGrid

    /terrain/entropy

    std_msgs/msg/Float32

---

## Gating Outputs

    /perception/gate_decision

    std_msgs/msg/String

    /perception/semantic_trigger

    std_msgs/msg/Bool

---

## State Output

    /state/estimated

    nav_msgs/msg/Odometry

---

# 31. Current ROS Package Structure

Current source structure:

    Perception/
    |
    +-- README.md
    |
    +-- matrix_perception/
        |
        +-- CMakeLists.txt
        +-- package.xml
        |
        +-- src/
            |
            +-- perception_test_node.cpp
            +-- stereo_test_publisher.cpp
            +-- camera_info_test_publisher.cpp
            +-- ground_plane_node.cpp
            +-- elevation_grid_node.cpp
            +-- roughness_node.cpp
            +-- clearance_node.cpp
            +-- step_height_node.cpp
            +-- terrain_test_publisher.cpp
            +-- traversability_costmap_node.cpp
            +-- entropy_node.cpp
            +-- entropy_test_publisher.cpp
            +-- gating_supervisor_node.cpp
            +-- iakf_fusion_node.cpp

---

# 32. Implemented Components

| Component | Status |
|---|---|
| ROS 2 Jazzy perception workspace | COMPLETE |
| matrix_perception package | COMPLETE |
| Basic ROS 2 perception node | COMPLETE |
| stereo_image_proc | COMPLETE |
| SGBM configuration | COMPLETE |
| Synthetic stereo fixture | COMPLETE |
| Stereo test publisher | COMPLETE |
| CameraInfo publisher | COMPLETE |
| Exact timestamp synchronization | COMPLETE |
| Disparity generation | COMPLETE |
| Disparity numerical validation | COMPLETE |
| Metric depth calculation | COMPLETE |
| Point-cloud generation | COMPLETE |
| PCL integration | COMPLETE |
| PointCloud2 → PCL conversion | COMPLETE |
| Finite XYZ filtering | COMPLETE |
| ROI processing | COMPLETE |
| RANSAC plane estimation | COMPLETE |
| SVD plane refinement | COMPLETE |
| Surface normal estimation | COMPLETE |
| Slope estimation | COMPLETE |
| Ground/non-ground separation | COMPLETE |
| Bayesian elevation mapping | COMPLETE |
| Elevation variance | COMPLETE |
| Terrain roughness | COMPLETE |
| Clearance estimation | COMPLETE |
| Step-height estimation | COMPLETE |
| Unified traversability costmap | COMPLETE |
| Geometric entropy | COMPLETE |
| Entropy gate | COMPLETE |
| Compact IAKF implementation | COMPLETE |
| Full physical sensor fusion | PLANNED |
| Camera-based entropy contrast term | PLANNED |
| Slip-cost augmentation | PLANNED |
| Semantic segmentation integration | PLANNED |
| Visual localization | PLANNED |
| Hybrid A* integration | PLANNED |
| DWA/local dynamic control | PLANNED |
| CBF-QP / MR-CBF safety layer | PLANNED |
| Physical UGV validation | PLANNED |

---

# 33. Planned Entropy-Gated Semantic Perception

A lightweight semantic branch will complement the deterministic geometric pipeline.

Conceptually:

    Camera
       |
       v
    Geometric / Uncertainty Assessment
       |
       +---------------------------+
       |                           |
       v                           v
    Geometry                   Geometry
    sufficient                 ambiguous
       |                           |
       v                           v
    Math-first                 SegFormer
      result                       |
                                  v
                           Semantic Information
                                  |
                                  v
                           Traversability Fusion

The current entropy gate demonstrates the control mechanism.

The semantic model itself is not part of the current Phase 3 perception implementation.

---

# 34. Planned Semantic Model

The planned lightweight semantic model is:

    SegFormer-B0

The semantic branch is intended to provide contextual information that cannot always be reliably obtained from geometry alone.

Potential classes include:

- Vegetation
- Mud
- Water
- Rough terrain
- Rubble
- Road
- Obstacles

The final class set depends on selected datasets and M.A.T.R.I.X-specific training.

A generic pretrained SegFormer model must not be assumed to directly produce final M.A.T.R.I.X terrain classes without suitable training or fine-tuning.

The repository also contains the separate PIDNet perception/training work contributed by the team.

The current Phase 3 mathematical terrain pipeline and the PIDNet semantic branch should be treated as complementary subsystems rather than the same implementation.

---

# 35. Planned Sensor Integration

The complete M.A.T.R.I.X perception architecture is intended to integrate:

    Stereo Camera
    16-beam LiDAR
    IMU
    Wheel Encoders
    RTK-GNSS

Stereo provides primary visual geometry.

LiDAR will provide complementary geometry for:

- Elevation
- Terrain structure
- Obstacle geometry
- Clearance
- Redundancy

IMU and wheel encoders will provide motion/state information.

RTK-GNSS may be used during development/reference operation where available.

The final architecture is intended to support GPS-denied navigation.

---

# 36. Planned Terrain Representation

The terrain representation is intended to become a local 2.5D grid containing layers such as:

- Elevation
- Elevation variance / uncertainty
- Surface slope
- Surface roughness
- Step height
- Clearance
- Traversability cost
- Semantic information

The current Phase 3 implementation already establishes the core mathematical terrain layers.

Established terrain mapping approaches may be used as algorithmic/reference sources, including:

- ANYbotics Elevation Mapping
- ANYbotics Grid Map
- Legged Robotics Traversability Estimation

Where an upstream repository is primarily ROS 1 or otherwise unsuitable as a direct ROS 2 dependency, it should be treated as an algorithmic/reference source rather than blindly integrated as a runtime dependency.

---

# 37. Planned Navigation Interface

After terrain perception is established, the traversability information will be consumed by the navigation architecture.

Planned chain:

    Traversability Costmap
            |
            v
    Hybrid A* / Smac Planner
            |
            v
       Global Path
            |
            v
        DWA / Local
       Dynamic Control
            |
            v
      Proposed cmd_vel
            |
            v
       CBF-QP / MR-CBF
       Safety Filtering
            |
            v
        /cmd_vel
            |
            v
       UGV Controller

Navigation and safety-control components are not yet implemented in the current `matrix_perception` package.

---

# 38. Integration Interfaces for Downstream Teammates

The Phase 3 perception subsystem is designed to provide the core outputs required by downstream navigation and safety components.

Important downstream interfaces include:

    /terrain/costmap
    /terrain/elevation
    /perception/obstacles
    /state/estimated
    /terrain/entropy

The semantic branch is expected to provide future outputs such as:

    /semantic/class_map
    /semantic/confidence

The navigation layer can consume:

    /terrain/costmap
    /terrain/elevation
    /perception/obstacles
    /state/estimated
    /terrain/entropy

The safety layer can additionally consume:

    /semantic/confidence

and filter the planner's proposed velocity command.

The final integrated system should maintain the command chain:

    Planner
       |
       v
    Proposed cmd_vel
       |
       v
    Safety Filter
       |
       v
    /cmd_vel
       |
       v
    UGV

---

# 39. Validation Strategy

The perception system is validated progressively.

## Level 1 — Mathematical Fixtures

Controlled synthetic/test inputs validate:

- Disparity
- Depth
- Point-cloud generation
- Ground plane
- Surface normal
- Slope
- Elevation
- Roughness
- Step height
- Traversability
- Entropy
- IAKF behavior

These fixtures provide controlled references.

## Level 2 — ROS 2 Integration

ROS interfaces are tested using:

    Publisher
       |
       v
    ROS Processing Node
       |
       v
    ROS Output

This verifies message compatibility and runtime execution.

## Level 3 — Dataset Replay

Real outdoor datasets will be introduced for realistic evaluation.

Potential datasets include:

- RELLIS-3D
- RUGD

## Level 4 — Simulation

The perception pipeline will be evaluated using the project simulation environment and real simulated sensor streams.

Ground-truth pose/terrain should not be substituted for perception outputs during acceptance testing.

## Level 5 — Physical UGV

The final validation stage will evaluate the complete perception/navigation system on the M.A.T.R.I.X compact 4WD UGV.

---

# 40. Development Performance

Current WSL2 CPU observations from the stereo foundation were:

    Disparity:
    approximately 2.8 - 3 Hz

    Point cloud:
    approximately 0.6 Hz

These measurements are not representative of final target hardware.

The intended deployment architecture targets NVIDIA Jetson-class hardware.

The broader architecture is designed around different processing rates:

    Fast:
    approximately 100 Hz
    state estimation / safety control

    Medium:
    approximately 20 Hz
    stereo geometry / local processing

    Slow:
    approximately 10 Hz
    terrain/elevation processing

    Asynchronous:
    approximately 2 - 5 Hz
    semantic perception

These are architectural targets rather than current measured performance for every component.

---

# 41. Reference Implementations

The perception architecture uses established robotics and computer-vision implementations as references.

## Stereo / Computer Vision

- ROS Image Pipeline / `stereo_image_proc`
- OpenCV

## Point Cloud / Geometry

- Point Cloud Library (PCL)

## Terrain Mapping

- ANYbotics Elevation Mapping
- ANYbotics Grid Map
- Legged Robotics Traversability Estimation

## Semantic Perception

- NVIDIA SegFormer
- NVIDIA Isaac ROS Image Segmentation
- PIDNet research/training implementation within the M.A.T.R.I.X repository

## State Estimation

- ROS 2 `robot_localization`

## Navigation

- ROS 2 Navigation2
- Smac Planner

## Safety / Optimization

- CBF-QP reference implementations
- ProxSuite

These projects are used as implementation and algorithm references.

M.A.T.R.I.X-specific functionality is implemented only after the required behavior and ROS 2 compatibility are understood and verified.

---

# 42. Current Development Status

## Phase 1 — ROS 2 Foundation

    ROS 2 Jazzy
         |
         v
    matrix_perception
         |
         v
    Basic ROS 2 Node

Status:

**COMPLETE**

---

## Phase 2 — Stereo Geometry

    Stereo Images
         |
         v
       SGBM
         |
         v
     Disparity
         |
         v
     Metric Depth
         |
         v
    Point Cloud

Status:

**COMPLETE — CONTROLLED SYNTHETIC VALIDATION**

---

## Phase 3 — Terrain Geometry & Traversability

    PointCloud2
         |
         v
    PCL Conversion
         |
         v
    Finite XYZ
         |
         v
    RANSAC
         |
         v
    SVD Refinement
         |
         v
    Ground Plane
         |
         v
       Slope
         |
         +----------------------+
         |                      |
         v                      v
    Ground/Elevation       Obstacles
         |
         +---------+------------+
                   |
                   v
           Terrain Features
                   |
        +----------+----------+
        |          |          |
        v          v          v
    Roughness  Step Height  Clearance
        |          |          |
        +----------+----------+
                   |
                   v
          Traversability Cost
                   |
                   v
                Costmap
                   |
                   v
            Entropy / Gate

Status:

**COMPLETE — CONTROLLED MATHEMATICAL / SYNTHETIC VALIDATION**

---

# 43. Phase 3 Limitations

The following limitations are explicitly retained:

### 1. Controlled fixtures

Most Phase 3 validation uses controlled synthetic/test geometry.

It does not constitute final outdoor field validation.

### 2. Development thresholds

Several parameters are engineering development values, including:

- Plane distance threshold
- Grid resolution
- Measurement variance
- Traversability weights
- Cost normalization thresholds
- Entropy threshold

These require tuning against real sensor data and UGV behavior.

### 3. Local slope representation

The current slope output is derived from the estimated dominant plane and represented across the local grid.

It is not yet a complete per-cell differential surface-normal estimator.

### 4. Slip augmentation

The architecture defines a slip-related traversability penalty, but it is not yet implemented.

### 5. Entropy

The current entropy implementation uses geometric elevation-variance information.

The final camera-contrast contribution is not yet implemented.

### 6. IAKF

The current IAKF is a compact validation implementation.

It is not yet the complete multi-sensor production estimator.

### 7. Physical sensor integration

Final integration with:

- Stereo hardware
- 16-beam LiDAR
- IMU
- Wheel encoders
- RTK-GNSS

remains future work.

### 8. Navigation and safety

The following remain future integration work:

- Hybrid A*
- DWA
- CBF-QP
- MR-CBF
- Final `/cmd_vel` safety chain

---

# 44. Engineering Rules

The perception development follows these rules:

1. Do not claim an algorithm is implemented until executable code exists.
2. Do not claim real-world validation using synthetic data.
3. Record numerical validation results whenever possible.
4. Keep deterministic mathematical processing as the primary perception path.
5. Introduce neural models only where they provide useful additional information.
6. Keep ROS interfaces explicit and independently testable.
7. Validate each stage before connecting it to the next stage.
8. Reuse mature ROS 2 infrastructure where appropriate.
9. Clearly distinguish development-environment performance from target-hardware performance.
10. Preserve teammate contributions when synchronizing the shared repository.
11. Maintain reproducible development fixtures for mathematical validation.
12. Do not treat a development fixture as equivalent to physical outdoor terrain.
13. Do not use simulator ground truth as a substitute for perception outputs during acceptance testing.
14. Keep mathematical validation, semantic perception, navigation, and safety control as independently testable components.
15. Document known limitations instead of overstating prototype capability.

---

# 45. Git / Repository Synchronization

The perception implementation is maintained in the shared:

    Perceptronix/M.A.T.R.I.X

repository.

The perception implementation is located under:

    Perception/matrix_perception/

The repository also contains:

    PIDNet/

The PIDNet work and mathematical perception work are maintained as complementary components.

The Phase 3 perception milestone was committed and synchronized with the latest remote repository history without force-pushing or overwriting teammate work.

Current Phase 3 milestone commit:

    36438c9

Commit message:

    feat(perception): complete Phase 3 terrain pipeline

The Phase 3 implementation was rebased onto the latest shared `main` branch before being pushed.

---

# 46. Current Milestone Checklist

## Phase 1 — ROS 2 Foundation

- [x] ROS 2 Jazzy environment
- [x] ROS 2 workspace
- [x] matrix_perception package
- [x] Basic perception node
- [x] Build validation
- [x] Runtime validation

## Phase 2 — Stereo Geometry

- [x] stereo_image_proc
- [x] SGBM
- [x] Synthetic stereo fixture
- [x] Stereo test publisher
- [x] CameraInfo publication
- [x] Exact timestamp synchronization
- [x] Disparity generation
- [x] Disparity numerical validation
- [x] Metric depth
- [x] Point-cloud generation
- [x] Point-cloud numerical validation

## Phase 3 — Ground Plane & Terrain Geometry

- [x] PointCloud2 subscription
- [x] PointCloud2 → PCL conversion
- [x] Finite XYZ filtering
- [x] ROI processing
- [x] RANSAC plane segmentation
- [x] SVD plane refinement
- [x] Surface normal estimation
- [x] Slope estimation
- [x] Ground/non-ground separation
- [x] Bayesian elevation
- [x] Elevation variance
- [x] Roughness
- [x] Clearance
- [x] Step height
- [x] Unified traversability costmap
- [x] Geometric entropy
- [x] Entropy gate
- [x] Compact IAKF implementation
- [x] Controlled validation
- [x] Clean ROS 2 build

## Future Integration

- [ ] Complete IMU/encoder/GNSS fusion
- [ ] Physical stereo camera integration
- [ ] Physical LiDAR integration
- [ ] Camera-based entropy contrast
- [ ] Slip-cost augmentation
- [ ] Semantic segmentation integration
- [ ] SegFormer/PIDNet integration with entropy gate
- [ ] Visual localization
- [ ] Hybrid A* integration
- [ ] DWA/local dynamic control
- [ ] CBF-QP safety filter
- [ ] MR-CBF implementation
- [ ] End-to-end simulation validation
- [ ] Physical UGV validation

---

# 47. Summary

The M.A.T.R.I.X perception subsystem has progressed from a basic ROS 2 package to a validated mathematical terrain-perception pipeline.

The current validated pipeline is:

    Stereo Geometry
         |
         v
      Disparity
         |
         v
     Metric Depth
         |
         v
     3D Point Cloud
         |
         v
      PointCloud2
         |
         v
         PCL
         |
         v
    Finite XYZ Filtering
         |
         v
    Ground Plane
    RANSAC + SVD
         |
         v
       Slope
         |
         +------------------+
         |                  |
         v                  v
    Elevation           Obstacles
         |
         v
    Elevation Variance
         |
    +----+-----+----------+
    |          |          |
    v          v          v
 Roughness  Step Height Clearance
    |          |          |
    +----------+----------+
               |
               v
      Traversability Costmap
               |
               v
         Geometric Entropy
               |
               v
          Entropy Gate

Parallel state estimation:

    Measurements
         |
         v
       IAKF
         |
         v
    /state/estimated

The resulting terrain representation is intended to support:

    Terrain Understanding
           |
           v
    Traversability Estimation
           |
           v
    Localization
           |
           v
    Path Planning
           |
           v
    Local Control
           |
           v
    CBF Safety Filtering
           |
           v
    Autonomous UGV Navigation

The final autonomous navigation architecture is not yet complete.

The next development focus is **integration of the validated Phase 3 perception outputs with the semantic, navigation, and safety subsystems**, followed by realistic simulation and physical UGV validation.

---

# 48. Current Project Status

**Completed milestone:**

    PHASE 3 — MATHEMATICAL TERRAIN PERCEPTION

Validated components:

    PointCloud2
        ↓
    PCL
        ↓
    Ground Plane
        ↓
    RANSAC + SVD
        ↓
    Slope
        ↓
    Bayesian Elevation
        ↓
    Roughness
        ↓
    Clearance
        ↓
    Step Height
        ↓
    Traversability Costmap
        ↓
    Geometric Entropy
        ↓
    Entropy Gate

Parallel:

    IAKF
      ↓
    /state/estimated

Current milestone commit:

    36438c9

Current next stage:

    PHASE 4 — PERCEPTION INTEGRATION

Primary objectives:

    Mathematical Terrain
          |
          +------------------+
          |                  |
          v                  v
      Semantic          Navigation
      Perception         Planning
          |                  |
          +--------+---------+
                   |
                   v
             Safety Layer
                   |
                   v
             Autonomous UGV
