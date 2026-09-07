"""Launches the full stack: the four comm_server-replacement bridge nodes, plus the four
legacy planner binaries they talk to.

Data flow: octomap_server voxelizes the camera's point cloud into an octree and publishes it as
octomap_binary; octomap_to_voxelmap deserializes that octree into the fixed 100x30x100 @ 0.2m
dense layout the legacy binaries hardcode and republishes it as /ugv/map (see
src/perception/octomap_to_voxelmap_node.cpp for the exact index/byte-order/cell-code contract).
goal_generation_bridge publishes /ugv/goal, which path_planner_bridge consumes and turns into
/ugv/path, which trajectory_planner_bridge consumes and turns into /ugv/trajectory (state) and
/ugv/control_sequence (F_x, delta_f). constraint_generation_bridge consumes /ugv/map + /ugv/pose
(same as goal_generation) and publishes /ugv/constraints, which trajectory_planner_bridge also
consumes. /ugv/pose and the camera's own point-cloud/driver stack (e.g. a ZED wrapper launch)
are still expected to come from elsewhere -- they are not started by this file. Until something
publishes /ugv/pose, and until octomap_server is actually receiving a point cloud, the
corresponding legacy binaries will simply block waiting for that first message, exactly as they
already block waiting for a TCP connection.

octomap_server's `frame_id` below MUST be set to whatever frame the camera's own driver/wrapper
publishes as its stable, start-anchored world frame (commonly "odom") -- octomap_to_voxelmap does
not track vehicle pose itself, it relies on octomap_server having already transformed points into
that frame via tf, so that frame's origin is the vehicle's start location. `cloud_in` must be
remapped to whatever topic that camera driver actually publishes its point cloud on. Both are
placeholders here and need to be confirmed against the real camera setup before this will work.

The legacy binaries (goal_generation, path_planner, fmpc_uncut, constraint_generation) are built
by each package's own plain CMakeLists.txt under
tactical_ugv_autonomous_stack/src/<package>/build/, not by this ament_cmake package's colcon
build -- there is no ROS-standard way to locate them from an installed package's own metadata,
so their location is derived below from this launch file's own installed path instead of being
hardcoded to one developer's machine (a previous version hardcoded it, which silently broke
every ExecuteProcess below on any other machine/checkout path). Each binary busy-loops
connect() until its bridge node is listening, so start order between a bridge and its binary
doesn't matter -- but you do need to have actually built all four first (cmake .. && make in
each package's own build/ directory) or the corresponding ExecuteProcess will fail to find its
executable. constraint_generation additionally needs libatlas-base-dev, libsdpa-dev,
libmumps-seq-dev, and liblapack-dev installed before it will build and link.
"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# This file installs to <workspace>/install/tactical_ugv_autonomous_stack/share/
# tactical_ugv_autonomous_stack/launch/bridge_nodes.launch.py -- six directories up from here is
# <workspace>, and the legacy binaries always live under
# <workspace>/tactical_ugv_autonomous_stack/src/<package>/build/, regardless of which machine or
# path the workspace was cloned to.
_THIS_FILE = os.path.abspath(__file__)
_WORKSPACE_ROOT = os.path.abspath(os.path.join(_THIS_FILE, *([os.pardir] * 6)))
_STACK_SRC = os.path.join(_WORKSPACE_ROOT, 'tactical_ugv_autonomous_stack', 'src')

if not os.path.isdir(os.path.join(_STACK_SRC, 'trajectory_planner')):
    raise RuntimeError(
        "bridge_nodes.launch.py could not find the legacy binaries' source tree at "
        f"'{_STACK_SRC}' (derived from this launch file's own path, "
        f"'{_THIS_FILE}'). If the install layout ever changes, adjust the "
        "os.pardir count above to match."
    )


def generate_launch_description():
    return LaunchDescription([
        # --- Perception: point cloud -> octree -> fixed dense VoxelMap on /ugv/map ---
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'resolution': 0.2,
                # MUST match the camera driver's stable, start-anchored world frame.
                'frame_id': 'base_link',
                'pointcloud_min_x': -10.0,
                'pointcloud_max_x': 10.0,
                'pointcloud_min_y': -10.0,
                'pointcloud_max_y': 10.0,
                'pointcloud_min_z': 0.0,
                'pointcloud_max_z': 6.0,
            }],
            remappings=[
                # MUST match whatever topic the camera driver publishes its point cloud on.
                ('cloud_in', '/zed/zed_node/point_cloud/cloud_registered'),
            ],
        ),
        Node(
            package='tactical_ugv_autonomous_stack',
            executable='octomap_to_voxelmap',
            name='octomap_to_voxelmap',
            output='screen',
        ),

        # --- ROS2 bridge nodes (comm_server replacement) ---
        Node(
            package='tactical_ugv_autonomous_stack',
            executable='goal_generation_bridge',
            name='goal_generation_bridge',
            output='screen',
        ),
        Node(
            package='tactical_ugv_autonomous_stack',
            executable='path_planner_bridge',
            name='path_planner_bridge',
            output='screen',
        ),
        Node(
            package='tactical_ugv_autonomous_stack',
            executable='trajectory_planner_bridge',
            name='trajectory_planner_bridge',
            output='screen',
        ),
        Node(
            package='tactical_ugv_autonomous_stack',
            executable='constraint_generation_bridge',
            name='constraint_generation_bridge',
            output='screen',
        ),

        # --- Legacy planner binaries, each run with cwd set to its own build/ directory so
        # its relative "Parameter_Files/..." reads resolve correctly. ---
        ExecuteProcess(
            cmd=[_STACK_SRC + '/goal_generation/build/goal_generation'],
            cwd=_STACK_SRC + '/goal_generation/build',
            name='goal_generation',
            output='screen',
        ),
        ExecuteProcess(
            cmd=[_STACK_SRC + '/path_planner/build/path_planner'],
            cwd=_STACK_SRC + '/path_planner/build',
            name='path_planner',
            output='screen',
        ),
        ExecuteProcess(
            cmd=[_STACK_SRC + '/trajectory_planner/build/fmpc_uncut'],
            cwd=_STACK_SRC + '/trajectory_planner/build',
            name='fmpc_uncut',
            output='screen',
        ),
        ExecuteProcess(
            cmd=[_STACK_SRC + '/constraint_generation/build/constraint_generation'],
            cwd=_STACK_SRC + '/constraint_generation/build',
            name='constraint_generation',
            output='screen',
        ),
    ])
