"""M.A.T.R.I.X full live demonstration: Member 1 (frozen, unmodified) +
Member 2 (real upstream perception pipeline) + Member 3 (real trained
PIDNet-S ONNX) + Member 4 (Hybrid A* + DWA, first working implementation,
authored this session) + Member 5 (CBF safety filter, first working
implementation, authored this session) + RViz2 + the live dashboard.

Member 4/5 honesty note: no Hybrid A*/DWA/CBF code existed anywhere in
this repository before this session. What is launched here is a genuine,
first-pass, receding-horizon implementation (Member 2 provides only a
rolling ~6x6m robot-relative local costmap -- no persistent global map or
SLAM exists in this stack, since GPS is deliberately off and no
localization member is integrated), not a placeholder. See
matrix_navigation/*.py docstrings and the acceptance report for exactly
what is and is not implemented.

UGV spawn: (x=6.1, y=0.1) on Rubicon's own heightmap -- physics-validated,
see prior report addenda for the full derivation.

Usage:
    cd ~/matrix_working
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch matrix_demo matrix_demo.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ugv_sim_share = get_package_share_directory('ugv_sim')
    matrix_perception_share = get_package_share_directory('matrix_perception')
    matrix_semantic_share = get_package_share_directory('matrix_semantic')
    matrix_navigation_share = get_package_share_directory('matrix_navigation')
    matrix_demo_share = get_package_share_directory('matrix_demo')

    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='5.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='0.0')
    goal_tol_arg = DeclareLaunchArgument('goal_tolerance_m', default_value='0.4')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_sim_share, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'world': 'fuel_rubicon_world',
            'start_x': '6.1',
            'start_y': '0.1',
            'start_z': '2.0',
            'start_yaw': '0.0',
        }.items()
    )

    perception = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(matrix_perception_share, 'launch', 'perception.launch.py')))]
    )

    # with_test_motion_adapter is now false by default: Member 4/5 own
    # /cmd_vel. The test adapter remains available (semantic.launch.py
    # with_test_motion_adapter:=true) for isolated M2+M3-only comparisons.
    semantic = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(matrix_semantic_share, 'launch', 'semantic.launch.py')),
            launch_arguments={'with_test_motion_adapter': 'false'}.items())]
    )

    navigation = TimerAction(
        period=13.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(matrix_navigation_share, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'goal_tolerance_m': LaunchConfiguration('goal_tolerance_m'),
            }.items())]
    )

    rviz = TimerAction(
        period=10.0,
        actions=[Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', os.path.join(matrix_demo_share, 'rviz', 'matrix_demo.rviz')])]
    )

    dashboard = Node(
        package='matrix_dashboard', executable='dashboard_node', output='screen')

    elevation_cloud_bridge = TimerAction(
        period=9.0,
        actions=[Node(
            package='matrix_dashboard', executable='elevation_cloud_bridge', output='screen')]
    )

    return LaunchDescription([
        goal_x_arg,
        goal_y_arg,
        goal_tol_arg,
        simulation,
        perception,
        semantic,
        navigation,
        rviz,
        dashboard,
        elevation_cloud_bridge,
    ])
