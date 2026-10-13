import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory('ugv_sim')

    # 'outdoor' (default) is the original small flat test course
    # (worlds/outdoor.world.sdf) -- preserved, unchanged. 'fuel_rubicon_world'
    # is a downloaded Gazebo Fuel world (OpenRobotics' "Rubicon" off-road
    # terrain: 37.5x37.5 m heightmap, rocks, logs, a ranger-station
    # building; see worlds/fuel/rubicon_world/), copied into the project
    # rather than left in the user's Fuel cache so the package doesn't
    # depend on an unmanaged temporary location.
    world_arg = DeclareLaunchArgument(
        'world', default_value='outdoor',
        description="'outdoor' (flat test course) or 'fuel_rubicon_world'"
    )
    start_x_arg = DeclareLaunchArgument('start_x', default_value='0.0')
    start_y_arg = DeclareLaunchArgument('start_y', default_value='0.0')
    start_z_arg = DeclareLaunchArgument('start_z', default_value='0.5')
    start_yaw_arg = DeclareLaunchArgument('start_yaw', default_value='0.0')

    world = LaunchConfiguration('world')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_z = LaunchConfiguration('start_z')
    start_yaw = LaunchConfiguration('start_yaw')

    outdoor_world_file = os.path.join(pkg_path, 'worlds', 'outdoor.world.sdf')
    rubicon_world_file = os.path.join(
        pkg_path, 'worlds', 'fuel', 'rubicon_world', 'rubicon.sdf')
    rubicon_models_dir = os.path.join(
        pkg_path, 'worlds', 'fuel', 'rubicon_world', 'models')

    world_file = PythonExpression([
        "'", rubicon_world_file, "' if '", world,
        "' == 'fuel_rubicon_world' else '", outdoor_world_file, "'"
    ])

    # Harmless when the outdoor world is selected (nothing in it resolves
    # model:// against this path); required for fuel_rubicon_world's
    # <uri>model://Rubicon</uri> include to find the locally-copied model.
    rubicon_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', rubicon_models_dir
    )

    robot_file = os.path.join(
        pkg_path,
        'urdf',
        'ugv.urdf.xacro'
    )

    # Process Xacro into URDF before sending it to ROS/Gazebo
    robot_description = {
        'robot_description': Command([
            'xacro ',
            robot_file
        ])
    }

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': [world_file, ' -r']
        }.items()
    )

    # Publish robot TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )
    # Bridge Gazebo transport topics to ROS 2. YAML config (rather than
    # plain "topic@type@type" CLI args) so the ground_truth_pose bridge
    # below can map a gz topic to a differently-named ROS topic.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_path, 'config', 'bridge.yaml'),
        }],
        output='screen'
    )
    # Spawn robot from /robot_description
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ugv',
            '-topic', 'robot_description',
            '-x', start_x,
            '-y', start_y,
            '-z', start_z,
            '-Y', start_yaw,
        ],
        output='screen'
    )

    # Fail-safe: stop the vehicle if /cmd_vel goes silent (see
    # cmd_vel_watchdog.py for why this is necessary — the DiffDrive plugin
    # has no built-in command timeout).
    cmd_vel_watchdog = Node(
        package='ugv_sim',
        executable='cmd_vel_watchdog',
        output='screen'
    )

    # SDFormat reduces every fixed-jointed link (lidar/camera/imu) into the
    # single physical link "base_link" when converting the URDF for Gazebo,
    # so gz-sim's sensor plugins report each sensor's frame_id as
    # "ugv/base_link/<sensor_name>" (the sensor's real physical parent in
    # the reduced SDF). robot_state_publisher, working from the
    # un-reduced URDF, publishes TF using the original link names
    # (lidar_link, camera_left_link, ...) instead, so sensor message
    # frame_ids do not otherwise exist anywhere in TF. These static
    # transforms alias the two naming schemes using the same physical
    # offsets already declared in ugv_sensors.xacro, so any consumer can
    # tf2-lookup a sensor message's own frame_id against base_link.
    sensor_frame_aliases = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0.63',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'ugv/base_link/lidar'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0.41',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'ugv/base_link/imu_sensor'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.42', '--y', '0.06', '--z', '0.45',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'ugv/base_link/camera_left'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0.42', '--y', '-0.06', '--z', '0.45',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'ugv/base_link/camera_right'],
        ),
    ]

    return LaunchDescription([
        world_arg,
        start_x_arg,
        start_y_arg,
        start_z_arg,
        start_yaw_arg,
        rubicon_resource_path,
        gazebo,
        robot_state_publisher,
        bridge,
        spawn_robot,
        cmd_vel_watchdog,
        *sensor_frame_aliases
    ])
