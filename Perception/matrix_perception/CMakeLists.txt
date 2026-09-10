cmake_minimum_required(VERSION 3.8)
project(matrix_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters sample_consensus segmentation)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(perception_test_node
  src/perception_test_node.cpp
)

ament_target_dependencies(perception_test_node
  rclcpp
)


add_executable(entropy_node
  src/entropy_node.cpp
)

ament_target_dependencies(entropy_node
  rclcpp
  std_msgs
)

add_executable(traversability_costmap_node
  src/traversability_costmap_node.cpp
)

ament_target_dependencies(traversability_costmap_node
  rclcpp
  std_msgs
  nav_msgs
)

add_executable(stereo_test_publisher
  src/stereo_test_publisher.cpp
)

ament_target_dependencies(stereo_test_publisher
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(stereo_test_publisher
  ${OpenCV_LIBS}
)

add_executable(camera_info_test_publisher
  src/camera_info_test_publisher.cpp
)

ament_target_dependencies(camera_info_test_publisher
  rclcpp
  sensor_msgs
)

add_executable(iakf_fusion_node
  src/iakf_fusion_node.cpp
)

ament_target_dependencies(iakf_fusion_node
  rclcpp
  geometry_msgs
  nav_msgs
)

target_link_libraries(iakf_fusion_node
  Eigen3::Eigen
)

add_executable(ground_plane_node src/ground_plane_node.cpp)

add_executable(elevation_grid_node
  src/elevation_grid_node.cpp
)

ament_target_dependencies(elevation_grid_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
)

add_executable(roughness_node
  src/roughness_node.cpp
)

add_executable(step_height_node
  src/step_height_node.cpp
)

ament_target_dependencies(step_height_node
  rclcpp
  std_msgs
)

add_executable(gating_supervisor_node
  src/gating_supervisor_node.cpp
)

ament_target_dependencies(gating_supervisor_node
  rclcpp
  std_msgs
)

add_executable(clearance_node
  src/clearance_node.cpp
)

ament_target_dependencies(roughness_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(clearance_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
)

add_executable(terrain_test_publisher
  src/terrain_test_publisher.cpp
)

ament_target_dependencies(terrain_test_publisher
  rclcpp
  sensor_msgs
)

add_executable(entropy_test_publisher
  src/entropy_test_publisher.cpp
)

ament_target_dependencies(entropy_test_publisher
  rclcpp
  std_msgs
)


ament_target_dependencies(ground_plane_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_include_directories(ground_plane_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(ground_plane_node
  ${PCL_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

install(TARGETS
  perception_test_node
  stereo_test_publisher
  camera_info_test_publisher
  ground_plane_node
  elevation_grid_node
  roughness_node
  iakf_fusion_node
  clearance_node
  step_height_node
  traversability_costmap_node
  entropy_test_publisher
  entropy_node
  terrain_test_publisher
  gating_supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
