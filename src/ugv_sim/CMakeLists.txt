cmake_minimum_required(VERSION 3.8)
project(ugv_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- Member 1 C++ runtime node -------------------------------------------
# cmd_vel_watchdog is the only Member-1-owned runtime ROS node (the
# /cmd_vel deadman safety fail-safe, responsibility #9 in Member 1's
# scope). Migrated from Python to C++ per the Member 1 engineering audit;
# see MEMBER_1_ACCEPTANCE_REPORT.md and CPP_MIGRATION_REPORT.md.
add_executable(cmd_vel_watchdog src/cmd_vel_watchdog.cpp)
target_compile_features(cmd_vel_watchdog PUBLIC cxx_std_17)
ament_target_dependencies(cmd_vel_watchdog rclcpp geometry_msgs)

install(TARGETS
  cmd_vel_watchdog
  DESTINATION lib/${PROJECT_NAME}
)

# --- Member 4 / legacy Python (untouched, out of scope for this pass) ----
# ugv_sim/navigation/* (Hybrid A* + DWA) and ugv_sim/goal_navigator.py are
# explicitly NOT part of Member 1's responsibilities and are installed
# as-is. The `ros2 run ugv_sim <name>` entry points that setup.py's
# console_scripts used to provide are preserved via the wrapper scripts in
# scripts/ (ament_cmake has no console_scripts equivalent).
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/navigation_node
  scripts/goal_navigator
  DESTINATION lib/${PROJECT_NAME}
)

# --- Configuration / description files (unchanged content/format) --------
install(DIRECTORY
  launch
  urdf
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
