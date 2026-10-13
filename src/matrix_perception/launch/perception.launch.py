"""M.A.T.R.I.X Member-2 perception launch (real upstream pipeline).

Wires Perceptronix/M.A.T.R.I.X @ b17e5fc's actual Member-2 nodes
(ground_plane_node, elevation_grid_node, roughness_node, step_height_node,
clearance_node, entropy_node, gating_supervisor_node,
traversability_costmap_node) to Member-1's REAL Gazebo sensors -- never
Gazebo ground truth.

Stereo depth (Z=fB/d) still comes from the stock stereo_image_proc
package (matched to Member-1's live CameraInfo, see depth_node.cpp) plus
this integration's own depth_node/state_estimation_node, neither of which
exists upstream.

Frame fix (this integration's addition, upstream nodes unmodified):
ground_plane_node/elevation_grid_node/roughness_node/clearance_node all
index a fixed local grid directly off an incoming cloud's x/y with no
internal TF lookup -- i.e. they expect a REP-103 (X-forward/Y-left/Z-up)
frame, but stereo_image_proc's real /points2 is in the camera's OPTICAL
frame. points2_frame_bridge performs the one missing transform via real,
live TF and republishes on /points2/base_link; the four consumers are
pointed at it purely by ROS remapping below, with no source changes.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    stereo_image_proc_launch = os.path.join(
        get_package_share_directory('stereo_image_proc'),
        'launch',
        'stereo_image_proc.launch.py'
    )

    stereo_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stereo_image_proc_launch),
        launch_arguments={
            'left_namespace': 'camera/left',
            'right_namespace': 'camera/right',
            'stereo_algorithm': '1',  # SGBM
            'approximate_sync': 'true',
        }.items()
    )

    depth_node = Node(package='matrix_perception', executable='depth_node', output='screen')

    state_estimation_node = Node(
        package='matrix_perception', executable='state_estimation_node', output='screen')

    points2_frame_bridge = Node(
        package='matrix_perception', executable='points2_frame_bridge', output='screen')

    points2_remap = [('/points2', '/points2/base_link')]

    ground_plane_node = Node(
        package='matrix_perception', executable='ground_plane_node', output='screen',
        remappings=points2_remap)

    elevation_grid_node = Node(
        package='matrix_perception', executable='elevation_grid_node', output='screen',
        remappings=points2_remap)

    roughness_node = Node(
        package='matrix_perception', executable='roughness_node', output='screen',
        remappings=points2_remap)

    clearance_node = Node(
        package='matrix_perception', executable='clearance_node', output='screen',
        remappings=points2_remap)

    step_height_node = Node(
        package='matrix_perception', executable='step_height_node', output='screen')

    entropy_node = Node(
        package='matrix_perception', executable='entropy_node', output='screen')

    # Intended threshold per gating_supervisor_node.cpp's own declared
    # default (0.30) against the bounded [0,1] entropy from entropy_node --
    # this is the real, documented threshold, not an arbitrary value.
    gating_supervisor_node = Node(
        package='matrix_perception', executable='gating_supervisor_node', output='screen',
        parameters=[{'entropy_threshold': 0.30}])

    traversability_costmap_node = Node(
        package='matrix_perception', executable='traversability_costmap_node', output='screen')

    return LaunchDescription([
        stereo_proc,
        depth_node,
        state_estimation_node,
        points2_frame_bridge,
        ground_plane_node,
        elevation_grid_node,
        roughness_node,
        clearance_node,
        step_height_node,
        entropy_node,
        gating_supervisor_node,
        traversability_costmap_node,
    ])
