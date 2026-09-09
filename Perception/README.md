# M.A.T.R.I.X — Perception

## SIH26126 — Vision Based Autonomous Navigation for Unmanned Ground Vehicle for Outdoor Environment

This directory contains the **Perception subsystem** of M.A.T.R.I.X, being developed for **Smart India Hackathon 2026 — SIH26126**, focused on vision-based autonomous navigation of an Unmanned Ground Vehicle (UGV) in outdoor environments.

The perception subsystem converts raw sensor observations into structured geometric and terrain information that can later be consumed by localization, traversability estimation, path planning, and safety control.

The development follows a **mathematics-first, deterministic perception architecture**, with lightweight semantic perception introduced where geometric reasoning alone is insufficient.

> **Current implementation status:** ROS 2 Jazzy workspace, stereo geometry pipeline, disparity validation, 3D point-cloud generation, PCL integration, and finite XYZ filtering are implemented and validated. Ground-plane estimation and terrain interpretation are currently under development.

---

# 1. Project Overview

The M.A.T.R.I.X perception subsystem is responsible for transforming raw sensor measurements into information about the surrounding outdoor environment.

The long-term perception pipeline is:

    Stereo Camera
          |
          v
    Stereo Processing
          |
          +----------------+
          |                |
      Disparity        Optical Flow
          |                |
          +--------+-------+
                   |
                   v
              3D Geometry
                   |
          +--------+--------+
          |                 |
     Ground Plane       Non-Ground
     Estimation           Objects
          |
          v
    Terrain Features
          |
     +----+----+----+
     |         |    |
   Slope   Roughness Clearance
     |         |    |
     +---------+----+
               |
               v
       Traversability
               |
               v
        Navigation Stack

The complete M.A.T.R.I.X architecture will additionally integrate:

- Stereo camera
- LiDAR
- IMU
- Wheel encoders
- RTK-GNSS
- Sensor fusion
- Elevation mapping
- Traversability estimation
- Lightweight semantic segmentation
- Visual localization
- Hybrid A* planning
- Local dynamic control
- CBF-based safety filtering

Only components explicitly marked as implemented in this README should be considered completed.

---

# 2. Development Philosophy

The perception system is being developed incrementally.

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

Every stage is independently tested before being connected to the next stage.

The project uses the following status definitions:

| Status | Meaning |
|---|---|
| IMPLEMENTED | Code exists and can be executed |
| VALIDATED | Output has been numerically or behaviorally verified |
| IN PROGRESS | Implementation is actively being developed |
| PLANNED | Part of the architecture but not implemented yet |
| DEVELOPMENT FIXTURE | Controlled synthetic data used for validation |

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
- ROS stereo processing: stereo_image_proc

Workspace:

    ~/matrix_ws

Main perception package:

    matrix_perception

GitHub working copy:

    ~/matrix_github

Repository perception directory:

    Perception/

---

# 4. Phase 1 — ROS 2 Perception Workspace Setup

## Status

**COMPLETE**

Phase 1 established the ROS 2 perception development environment and verified that a custom M.A.T.R.I.X perception package could be created, compiled, installed, and executed.

## 4.1 ROS 2 Package

The main package is:

    matrix_perception

The package uses the ROS 2:

    ament_cmake

build system.

Initial ROS dependencies include:

- rclcpp
- sensor_msgs
- geometry_msgs
- cv_bridge
- image_transport

Additional dependencies were added as the perception pipeline progressed.

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

The basic ROS 2 perception foundation was successfully established.

Validation:

- ROS 2 Jazzy environment — PASS
- ROS workspace — PASS
- matrix_perception package — PASS
- ament_cmake build — PASS
- C++ perception node — PASS
- ROS 2 runtime execution — PASS

**Phase 1: COMPLETE**

---

# 5. Phase 2 — Stereo Geometry Foundation

## Status

**COMPLETE — CONTROLLED SYNTHETIC VALIDATION**

Phase 2 established the first complete perception pipeline from stereo images to metric 3D point-cloud data.

Pipeline:

    Left Image
         |
         |
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

The purpose of this phase was to verify the stereo geometry chain before using the generated 3D data for terrain processing.

---

# 6. Phase 2.1 — Stereo Image Processing

The ROS 2 package:

    stereo_image_proc

was installed and verified.

Available executables were confirmed:

    stereo_image_proc disparity_node
    stereo_image_proc point_cloud_node

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

The simulator did not yet provide usable stereo camera topics during this development stage.

Therefore, a controlled synthetic stereo fixture was created.

The fixture consists of:

    left.png
    right.png

Image resolution:

    640 x 480 pixels

Known synthetic disparity:

    20 pixels

Stereo parameters:

    Focal length:
    f = 500 pixels

    Baseline:
    B = 0.12 m

The standard stereo depth relationship is:

    Z = fB / d

Using:

    f = 500
    B = 0.12 m
    d = 20 pixels

the expected depth is:

    Z = 3.0 m

This known ground truth allowed the stereo processing pipeline to be numerically validated.

---

# 8. Phase 2.3 — Stereo Test Publisher

A dedicated test publisher was implemented:

    stereo_test_publisher.cpp

The publisher loads the synthetic stereo images and publishes:

    /left/image_raw
    /right/image_raw

    /left/camera_info
    /right/camera_info

An important synchronization issue was discovered during development.

Initially, the images and camera information were published with independent timestamps.

Because exact synchronization was being used, stereo_image_proc could not correctly pair the corresponding messages.

The publisher was corrected so that:

    Left Image
    Right Image
    Left CameraInfo
    Right CameraInfo

all use the same timestamp.

This allowed exact-time synchronization to operate correctly.

---

# 9. Phase 2.4 — Stereo Camera Calibration

The controlled stereo fixture uses the following intrinsic matrix:

    K =
    [ 500   0   320 ]
    [   0 500   240 ]
    [   0   0     1 ]

The right camera projection matrix contains:

    P[3] = -60

This corresponds to:

    -fB = -(500 x 0.12)
        = -60

The controlled fixture assumes:

- Zero distortion
- Identity rotation between cameras
- Known stereo baseline

This simplified calibration is intended for mathematical pipeline validation rather than final physical-camera calibration.

---

# 10. Phase 2.5 — Disparity Validation

The generated disparity image was numerically inspected.

Validation results:

    Image resolution:
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

The measured disparity closely matches the known synthetic ground truth.

This confirms that the SGBM stereo pipeline correctly recovered the expected disparity for the controlled fixture.

---

# 11. Phase 2.6 — Depth Validation

The stereo depth equation is:

    Z = fB / d

For the controlled fixture:

    f = 500 px
    B = 0.12 m
    d = 20 px

Expected depth:

    Z = 3.000 m

The generated point cloud was numerically inspected.

Measured values:

    Mean Z:
    3.000 m

    Median Z:
    3.000 m

    Z range:
    approximately 2.972 m - 3.038 m

    Mean absolute depth error:
    0.000 m

Point-cloud properties:

    Resolution:
    640 x 480

    Point step:
    32 bytes

    Valid XYZ points:
    161572

The measured depth therefore agrees with the known synthetic ground truth.

---

# 12. Phase 2.7 — Development Performance Observation

During development under WSL2 using CPU processing:

    Disparity processing:
    approximately 2.8 - 3 Hz

    Point-cloud generation:
    approximately 0.6 Hz

These measurements are development-environment observations only.

They are NOT final M.A.T.R.I.X performance targets.

The final system is intended to use NVIDIA Jetson-class hardware with appropriate optimization and GPU acceleration.

---

# 13. Phase 2 Final Result

The complete stereo geometry chain was successfully demonstrated:

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
- Metric depth calculation — PASS
- Point-cloud generation — PASS
- Point-cloud numerical validation — PASS

**Phase 2: COMPLETE**

### Phase 2 Limitation

The validation used a synthetic stereo fixture.

Therefore, Phase 2 validates the stereo geometry implementation, but does not yet constitute final validation on the physical stereo camera or outdoor terrain.

---

# 14. Phase 3 — Ground Plane & Terrain Geometry

## Status

**IN PROGRESS**

Phase 3 converts the stereo-generated 3D point cloud into meaningful terrain geometry.

Target pipeline:

    /points2
       |
       v
    PointCloud2
       |
       v
    PCL Conversion
       |
       v
    Valid XYZ Filtering
       |
       v
    ROI Selection
       |
       v
    RANSAC Plane Segmentation
       |
       v
    SVD Plane Refinement
       |
       v
    Surface Normal
       |
       v
    Slope Estimation
       |
       v
    Ground / Non-Ground Separation

The first part of this pipeline has now been implemented and validated.

---

# 15. Phase 3.1 — Ground Plane ROS 2 Node

A dedicated terrain geometry node was created:

    ground_plane_node.cpp

The node subscribes to:

    /points2

using:

    sensor_msgs/msg/PointCloud2

and sensor-data QoS.

The incoming ROS point cloud is converted into:

    pcl::PointCloud<pcl::PointXYZ>

using:

    pcl_conversions

---

# 16. Phase 3.2 — PointCloud2 to PCL Conversion

The current implementation establishes the interface:

    ROS PointCloud2
          |
          v
    PCL PointCloud<PointXYZ>

This establishes the bridge between the ROS 2 stereo pipeline and the PCL-based terrain geometry algorithms.

PCL is being used for the planned:

- Point-cloud processing
- RANSAC plane segmentation
- Geometric filtering
- Terrain geometry extraction

The current node successfully receives /points2 and converts the incoming point cloud into a PCL representation.

---

# 17. Phase 3.3 — Valid XYZ Filtering

Stereo-generated point clouds contain invalid points wherever valid disparity/depth could not be established.

The current node explicitly checks every point for finite:

    X
    Y
    Z

coordinates.

A point is considered valid only when:

    isfinite(x)
    isfinite(y)
    isfinite(z)

This prevents NaN and infinite values from entering subsequent geometric processing.

---

# 18. Phase 3.4 — Point Cloud Validation

The current ground_plane_node was tested against the Phase-2 point cloud.

Observed result:

    Total points:
    307200

    Valid XYZ:
    161572

    Invalid:
    145628

Consistency check:

    161572 + 145628 = 307200

The number of valid XYZ points also matches the valid disparity population measured during Phase 2.

This confirms the current pipeline:

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

is functioning correctly.

---

# 19. Phase 3.5 — ROI Selection

## Status

**NEXT IMPLEMENTATION**

The next step is to introduce a physically meaningful spatial region of interest before plane fitting.

Target pipeline:

    Point Cloud
        |
        v
    Valid XYZ Filtering
        |
        v
    Coordinate Frame
        |
        v
    Spatial ROI
        |
        v
      RANSAC

The ROI will restrict processing to the portion of the environment relevant to the UGV's local terrain.

The ROI must be defined relative to an appropriate vehicle/sensor coordinate frame.

The complete point cloud should not automatically be assumed to represent the ground.

---

# 20. Phase 3.6 — RANSAC Ground Plane Estimation

## Status

**PLANNED**

After ROI selection, RANSAC plane segmentation will be implemented.

The plane representation is:

    ax + by + cz + d = 0

PCL sample-consensus plane segmentation will be used to identify the dominant planar surface inside the selected ROI.

RANSAC provides robustness against potential outliers including:

- Vegetation
- Obstacles
- Isolated depth errors
- Terrain discontinuities
- Non-ground structures

The plane distance threshold will determine whether a point is treated as an inlier.

---

# 21. Phase 3.7 — SVD Plane Refinement

## Status

**PLANNED**

After RANSAC identifies plane inliers, the plane estimate will be refined mathematically.

The inlier point distribution will be represented using its covariance structure.

The surface normal will be obtained from the eigenvector corresponding to the smallest covariance eigenvalue.

The resulting estimate will provide:

    Plane coefficients
    Surface normal
    Ground orientation

This follows the mathematical methodology defined for the M.A.T.R.I.X perception architecture.

---

# 22. Phase 3.8 — Slope Estimation

## Status

**PLANNED**

Once the ground-plane normal is available, terrain slope will be estimated from its orientation relative to the appropriate reference frame.

Conceptually:

    Ground Plane
         |
         v
    Surface Normal
         |
         v
    Orientation relative to reference vertical
         |
         v
       Slope

The slope value will become one of the primary terrain features used by the future traversability layer.

---

# 23. Phase 3.9 — Ground / Non-Ground Separation

## Status

**PLANNED**

The estimated ground plane will eventually be used to separate:

    Ground Points

from:

    Non-Ground Points

This separation will provide a geometric foundation for later obstacle and terrain interpretation.

---

# 24. Critical Synthetic Fixture Limitation

The Phase-2 synthetic point cloud represents a:

    Fronto-parallel plane
    Approximately 3 m from the camera

It was specifically designed to validate:

- Disparity
- Depth
- Point-cloud generation

It is NOT a physical representation of the UGV ground surface.

Therefore, the current fixture must not be used to claim meaningful:

- Ground-plane estimation
- Terrain slope estimation
- Outdoor terrain performance

Before validating RANSAC, SVD, and slope estimation, a dedicated controlled plane fixture will be created.

The fixture will contain a known plane orientation so that:

    Known Plane Orientation
             vs.
    Estimated Plane Orientation

can be compared numerically.

This prevents camera-facing synthetic geometry from being incorrectly interpreted as physical vehicle-ground geometry.

---

# 25. Current ROS Package Structure

The current perception repository structure is:

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

---

# 26. Implemented Components

| Component | Status |
|---|---|
| ROS 2 Jazzy perception workspace | COMPLETE |
| matrix_perception package | COMPLETE |
| Basic ROS 2 perception node | COMPLETE |
| stereo_image_proc installation | COMPLETE |
| SGBM configuration | COMPLETE |
| Synthetic stereo fixture | COMPLETE |
| Stereo test publisher | COMPLETE |
| CameraInfo publisher | COMPLETE |
| Exact timestamp synchronization | COMPLETE |
| Disparity generation | COMPLETE |
| Disparity numerical validation | COMPLETE |
| 3D point-cloud generation | COMPLETE |
| Depth numerical validation | COMPLETE |
| PCL integration | COMPLETE |
| PointCloud2 to PCL conversion | COMPLETE |
| Finite XYZ filtering | COMPLETE |
| ROI selection | NEXT |
| RANSAC plane estimation | PLANNED |
| SVD plane refinement | PLANNED |
| Surface normal estimation | PLANNED |
| Slope estimation | PLANNED |
| Ground/non-ground separation | PLANNED |
| Elevation mapping | PLANNED |
| Traversability estimation | PLANNED |
| Semantic segmentation | PLANNED |
| Visual localization | PLANNED |
| Navigation integration | PLANNED |
| CBF safety filtering | PLANNED |

---

# 27. Planned Full Perception Architecture

The long-term perception architecture is:

    Stereo Camera
          |
          +--------------------+
          |                    |
          v                    v
    Stereo Geometry       Optical Flow
          |                    |
          +---------+----------+
                    |
                    v
              3D Geometry
                    |
          +---------+---------+
          |                   |
          v                   v
     Ground Plane        Elevation Map
     RANSAC + SVD
          |                   |
          +---------+---------+
                    |
                    v
             Terrain Features
                    |
          +---------+---------+
          |         |         |
          v         v         v
        Slope   Roughness  Clearance
          |         |         |
          +---------+---------+
                    |
                    v
             Traversability
                    |
                    v
              Sensor Fusion
                    |
                    v
           Navigation / Planning

---

# 28. Planned Entropy-Gated Semantic Perception

A lightweight semantic branch is intended to complement the deterministic geometric pipeline.

The conceptual architecture is:

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

The purpose of this architecture is to avoid unnecessarily invoking neural perception when deterministic geometry is already sufficient.

Semantic inference is intended for ambiguous terrain/context cases where geometry alone may not provide enough information.

---

# 29. Planned Terrain Representation

The terrain representation is intended to become a local 2.5D grid containing layers such as:

- Elevation
- Variance / uncertainty
- Surface slope
- Surface roughness
- Obstacle / step height
- Clearance
- Traversability cost

The architecture references established terrain mapping approaches including:

- ANYbotics Elevation Mapping
- ANYbotics Grid Map
- Legged Robotics Traversability Estimation

Where an upstream repository is primarily ROS 1 or otherwise unsuitable as a direct ROS 2 dependency, it will be treated as an algorithmic/reference source rather than blindly integrated as a runtime dependency.

---

# 30. Planned Sensor Integration

The complete M.A.T.R.I.X perception architecture is intended to integrate:

    Stereo Camera
    16-beam LiDAR
    IMU
    Wheel Encoders
    RTK-GNSS

The stereo camera provides primary visual geometry.

LiDAR will provide complementary geometric information for:

- Elevation estimation
- Terrain structure
- Obstacle geometry
- Clearance
- Redundancy

IMU and wheel encoders will provide motion/state information for sensor fusion.

RTK-GNSS may be used for development/reference operation where available, while the final navigation architecture is intended to support GPS-denied operation.

---

# 31. Planned Semantic Perception

Semantic segmentation is a later-stage component.

The current planned lightweight model is:

    SegFormer-B0

The semantic branch is intended to provide contextual terrain information that cannot always be reliably obtained from geometry alone.

Potential terrain/context classes include:

- Vegetation
- Mud
- Water
- Rough terrain
- Rubble
- Road
- Obstacles

The final class set depends on the selected training datasets and M.A.T.R.I.X-specific annotation strategy.

A generic pretrained SegFormer model must not be assumed to directly produce the final M.A.T.R.I.X terrain classes without suitable training or fine-tuning.

---

# 32. Planned Traversability Estimation

Terrain geometry will eventually be converted into a unified traversability representation.

The planned cost representation incorporates factors including:

- Slope
- Roughness
- Obstacle / step height
- Clearance
- Uncertainty
- Semantic information

The resulting representation should allow the navigation system to distinguish between:

- Safe terrain
- Difficult terrain
- Uncertain terrain
- Non-traversable terrain

The objective is therefore not simply binary obstacle detection.

The system must understand whether terrain is suitable for UGV traversal.

---

# 33. Planned Navigation Interface

After terrain perception is established, the resulting traversability information will be integrated with the navigation architecture.

Planned components include:

    Smac Planner Hybrid
             |
             v
       Hybrid-A* Path
             |
             v
     Local Dynamic Control
             |
             v
       CBF-QP / MR-CBF
       Safety Filtering
             |
             v
       Vehicle Commands

Navigation and safety-control components are not yet implemented in the current perception package.

---

# 34. Validation Strategy

The perception system will be validated progressively.

## Level 1 — Mathematical Fixtures

Controlled synthetic inputs will be used to validate:

- Disparity
- Depth
- Plane geometry
- Surface normal
- Slope

These fixtures provide known ground truth.

## Level 2 — ROS 2 Integration

The actual ROS interfaces will be tested:

    ROS Publisher
         |
         v
    ROS Processing Node
         |
         v
    ROS Output

This verifies message compatibility and runtime integration.

## Level 3 — Dataset Replay

Real outdoor datasets will be introduced for realistic validation.

Potential datasets include:

- RELLIS-3D
- RUGD

## Level 4 — Simulation

The perception pipeline will be evaluated in an appropriate Gazebo/Ignition simulation environment.

## Level 5 — Physical UGV

The final validation stage will evaluate the perception system on the M.A.T.R.I.X compact 4WD UGV platform.

---

# 35. Reference Implementations

The perception architecture is based on established robotics and computer-vision implementations.

## Stereo / Computer Vision

- ROS Image Pipeline / stereo_image_proc
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

## State Estimation

- ROS 2 robot_localization

## Navigation

- ROS 2 Navigation2
- Smac Planner

## Safety / Optimization

- CBF-QP reference implementations
- ProxSuite

These repositories are treated as implementation and algorithm references.

M.A.T.R.I.X-specific functionality is implemented only after the required algorithmic behavior and ROS 2 compatibility are understood and verified.

---

# 36. Current Development Status

## Phase 1 — ROS 2 Foundation

Implemented:

    ROS 2 Workspace
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

Implemented:

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

**COMPLETE — SYNTHETIC VALIDATION**

---

## Phase 3 — Ground Plane & Terrain Geometry

Implemented:

    PointCloud2
         |
         v
    PCL Conversion
         |
         v
    Valid XYZ Filtering

Status:

**IN PROGRESS**

Next:

    Valid XYZ
         |
         v
       ROI
         |
         v
      RANSAC
         |
         v
       SVD
         |
         v
    Ground Plane
         |
         v
       Slope
         |
         v
    Ground / Non-Ground

---

# 37. Immediate Next Milestone

## Phase 3.5 — ROI Selection

The immediate next task is to implement a physically meaningful spatial ROI for terrain processing.

Target pipeline:

    /points2
       |
       v
    PointCloud2
       |
       v
    PCL Point Cloud
       |
       v
    Finite XYZ Filtering
       |
       v
    Coordinate Frame
       |
       v
    Spatial ROI
       |
       v
      RANSAC

The ROI will be validated before implementing RANSAC.

After ROI validation:

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
         v
    Ground / Non-Ground

will be implemented incrementally.

---

# 38. Engineering Rules

The perception development follows these rules:

1. Do not claim an algorithm is implemented until executable code exists.
2. Do not claim real-world validation using synthetic data.
3. Record numerical validation results whenever possible.
4. Keep deterministic mathematical processing as the primary perception path.
5. Introduce neural models only where they provide useful additional information.
6. Keep ROS interfaces explicit and independently testable.
7. Validate each stage before connecting it to the next stage.
8. Reuse mature ROS 2 infrastructure where appropriate instead of unnecessarily recreating it.
9. Clearly distinguish development-environment performance from target-hardware performance.
10. Preserve teammate contributions when synchronizing the shared repository.
11. Maintain reproducible development fixtures for mathematical validation.
12. Do not treat a development fixture as equivalent to physical outdoor terrain.

---

# 39. Git / Repository Synchronization

The perception development is maintained in the shared:

    Perceptronix/M.A.T.R.I.X

repository.

The perception implementation is located under:

    Perception/matrix_perception/

During synchronization with the shared repository, teammate work is preserved.

The repository currently contains both:

    PIDNet/
    Perception/

The local perception milestone was merged with the updated remote repository history without force-pushing or rewriting teammate commits.

The working tree was verified clean and synchronized with the shared main branch.

---

# 40. Summary

The M.A.T.R.I.X perception subsystem has progressed from a basic ROS 2 package to a validated stereo-to-3D geometry foundation.

The currently validated pipeline is:

    ROS 2 Jazzy
         |
         v
    Stereo Image Input
         |
         v
    Camera Information
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

The next stage is to transform the resulting 3D geometry into meaningful terrain information:

    3D Point Cloud
         |
         v
    ROI Selection
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
    Surface Normal
         |
         v
       Slope
         |
         v
    Terrain Geometry
         |
         v
    Traversability

The final objective is to provide a robust perception foundation for:

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

---

# 41. Current Milestone Checklist

### Phase 1 — ROS 2 Foundation

- [x] ROS 2 Jazzy environment
- [x] ROS 2 workspace
- [x] matrix_perception package
- [x] Basic perception node
- [x] Build and runtime validation

### Phase 2 — Stereo Geometry

- [x] stereo_image_proc installed
- [x] SGBM configured
- [x] Synthetic stereo fixture
- [x] Stereo test publisher
- [x] CameraInfo publication
- [x] Exact timestamp synchronization
- [x] Disparity generation
- [x] Disparity numerical validation
- [x] Metric depth validation
- [x] 3D point-cloud validation

### Phase 3 — Ground Plane & Terrain Geometry

- [x] PointCloud2 subscription
- [x] PointCloud2 → PCL conversion
- [x] Finite XYZ filtering
- [ ] ROI selection
- [ ] RANSAC plane segmentation
- [ ] SVD plane refinement
- [ ] Surface normal estimation
- [ ] Slope estimation
- [ ] Ground/non-ground separation

### Future Perception Stages

- [ ] Elevation mapping
- [ ] Terrain roughness
- [ ] Clearance estimation
- [ ] Traversability cost
- [ ] Adaptive sensor fusion
- [ ] Entropy gating
- [ ] SegFormer semantic branch
- [ ] Visual localization
- [ ] Navigation integration
- [ ] CBF-QP safety layer

---

# 42. Development Status

**Current active phase:**

    PHASE 3 — GROUND PLANE & TERRAIN GEOMETRY

**Current completed milestone:**

    PointCloud2
        ↓
    PCL Conversion
        ↓
    Finite XYZ Filtering

**Next milestone:**

    ROI Selection

**Following milestone:**

    RANSAC Ground Plane Estimation

The perception subsystem will continue to be developed and validated incrementally until the complete M.A.T.R.I.X autonomous navigation architecture is integrated.
