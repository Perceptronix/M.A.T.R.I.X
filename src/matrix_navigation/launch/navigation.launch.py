from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='5.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='0.0')
    goal_tol_arg = DeclareLaunchArgument('goal_tolerance_m', default_value='0.4')

    hybrid_astar = Node(
        package='matrix_navigation', executable='hybrid_astar_node', output='screen',
        parameters=[{
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'goal_tolerance_m': LaunchConfiguration('goal_tolerance_m'),
        }])
    dwa = Node(package='matrix_navigation', executable='dwa_node', output='screen')
    cbf = Node(package='matrix_navigation', executable='cbf_safety_node', output='screen')

    return LaunchDescription([goal_x_arg, goal_y_arg, goal_tol_arg, hybrid_astar, dwa, cbf])
