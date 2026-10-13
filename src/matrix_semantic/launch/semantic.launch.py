import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# PyTorch/onnxruntime/OpenCV are not installed in this machine's system
# Python (no apt/pip access at setup time -- see acceptance report). They
# were installed into an isolated venv created with --system-site-packages
# so it still sees rclpy/sensor_msgs/cv_bridge from the ROS 2 install.
# ros2 run's generated console-script entry point is pinned to the system
# python3 interpreter used at colcon-build time, so it can't pick up the
# venv -- semantic_node is launched by invoking the venv's python3 directly
# as a module instead. Everything else in this workspace is unaffected.
VENV_PYTHON = os.path.expanduser('~/matrix_torch_venv/bin/python3')


def generate_launch_description():
    motion_adapter_arg = DeclareLaunchArgument(
        'with_test_motion_adapter', default_value='false',
        description='Launch the TEMPORARY test-only motion adapter that maps '
                    '/terrain/costmap to /cmd_vel (NOT Member 4/5, verification-only).'
    )

    # No threshold parameter here: semantic_node no longer computes its own
    # gate. It subscribes to /perception/semantic_trigger, published by the
    # upstream repo's own gating_supervisor_node against entropy_node's
    # real bounded [0,1] entropy and that node's own declared default
    # threshold (0.30) -- launched as part of matrix_perception's
    # perception.launch.py, not duplicated here.
    semantic_node = ExecuteProcess(
        cmd=[VENV_PYTHON, '-m', 'matrix_semantic.semantic_node'],
        output='screen',
    )

    test_motion_adapter = Node(
        package='matrix_semantic',
        executable='test_motion_adapter',
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_test_motion_adapter')),
    )

    return LaunchDescription([
        motion_adapter_arg,
        semantic_node,
        test_motion_adapter,
    ])
