"""Launches the full stack: the four comm_server-replacement bridge nodes, plus the four
legacy planner binaries they talk to.

Data flow: octomap_server voxelizes the camera's point cloud into an octree and publishes it as
octomap_binary; octomap_to_voxelmap deserializes that octree into the fixed 100x30x100 @ 0.2m
dense layout the legacy binaries hardcode and republishes it as /ugv/map (see
src/perception/octomap_to_voxelmap_node.cpp for the exact index/byte-order/cell-code contract).
goal_generation_bridge publishes /ugv/goal, which path_planner_bridge consumes and turns into
/ugv/path, which trajectory_planner_bridge consumes and turns into /ugv/trajectory (state) and
/ugv/control_sequence ([vx, delta_f] -- NOT [F_x, delta_f]; see msg/ControlSequence.msg and
f_mpc_feedback_linearization.cpp's v_k for why F_x isn't usable on this vehicle).
control_sequence_to_ackermann consumes /ugv/control_sequence and republishes it as an
AckermannDriveStamped on /drive for the separately-launched f1tenth stack's /ackermann_mux to
arbitrate (see src/perception/control_sequence_to_ackermann_node.cpp for the full safety/rate
contract -- this is the only node in this launch file that can move the physical vehicle).
constraint_generation_bridge consumes /ugv/map + /ugv/pose (same as goal_generation) and
publishes /ugv/constraints, which trajectory_planner_bridge also consumes. /ugv/pose is provided
by zed_odom_to_legacy_pose, which republishes the ZED wrapper's odometry (/zed/zed_node/odom) as
LegacyPose (see src/perception/zed_odom_to_legacy_pose_node.cpp for the field-usage audit and the
finite-differencing used to derive velocity, since this installed ZED wrapper build never
populates the odometry message's twist). Only the camera's own point-cloud/driver stack (e.g. a
ZED wrapper launch) and the f1tenth stack (VESC driver, ackermann_mux, ackermann_to_vesc_node)
are still expected to come from elsewhere -- neither is started by this file. Until octomap_server
is actually receiving a point cloud and the ZED wrapper is actually publishing odometry, the
corresponding legacy binaries will simply block waiting for that first message, exactly as they
already block waiting for a TCP connection.

octomap_server's `frame_id` below MUST be set to whatever frame the camera's own driver/wrapper
publishes as its stable, start-anchored world frame -- octomap_to_voxelmap does not track vehicle
pose itself, it relies on octomap_server having already transformed points into that frame via
tf, so that frame's origin is the vehicle's start location. Confirmed against this system's live
ZED wrapper as "odom" (it publishes a dynamic odom -> zed_camera_link tf and never publishes a
base_link frame at all -- setting frame_id to a frame that doesn't exist makes octomap_server
silently fail to transform every cloud and never insert any points, which is what happened before
this was fixed). If the camera driver changes, re-confirm this against its actual tf tree
(`ros2 run tf2_ros tf2_echo <candidate_frame> <point_cloud's own frame_id>`) rather than assuming.
`cloud_in` is remapped below to this system's actual ZED point-cloud topic.

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
                # Confirmed against the live ZED wrapper's actual TF tree: it publishes odom as
                # the dynamic world frame (odom -> zed_camera_link -> ... -> camera optical
                # frames), and never publishes a base_link frame at all. octomap_server would
                # otherwise silently fail to transform every point cloud and never insert any
                # points, since canTransform() to a nonexistent frame always fails.
                'frame_id': 'odom',
                # Matches octomap_to_voxelmap's world<->voxel convention, which is in turn
                # matched to the legacy planner binaries' own (goal_generation/octree.h,
                # path_planner/LPAstar.cpp): corner-anchored at the vehicle's start, 0 to +20m
                # horizontally, 0 to +6m vertically -- NOT centered on start. A previous version
                # cropped -10..+10 here (centered), which silently discarded real detections
                # beyond 10m even after octomap_to_voxelmap was fixed to expect up to 20m.
                'pointcloud_min_x': 0.0,
                'pointcloud_max_x': 20.0,
                'pointcloud_min_y': 0.0,
                'pointcloud_max_y': 20.0,
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
        Node(
            package='tactical_ugv_autonomous_stack',
            executable='zed_odom_to_legacy_pose',
            name='zed_odom_to_legacy_pose',
            output='screen',
        ),

        # --- Actuation: /ugv/control_sequence -> /drive for the f1tenth stack's ackermann_mux.
        # max_speed_mps is the ONLY speed bound anywhere in this pipeline (the solver has no
        # vx_max constraint); max_steering_angle_rad matches this vehicle's real VESC servo
        # saturation (~0.29 rad), not the solver's own looser delta_f_max=0.5 rad bound. Tune
        # max_speed_mps deliberately before running with the physical drivetrain live -- there is
        # no dedicated e-stop on this vehicle, only the joystick deadman override on /teleop. ---
        Node(
            package='tactical_ugv_autonomous_stack',
            executable='control_sequence_to_ackermann',
            name='control_sequence_to_ackermann',
            output='screen',
            parameters=[{
                'control_sequence_topic': '/ugv/control_sequence',
                'drive_topic': '/drive',
                'max_speed_mps': 1.0,
                'max_steering_angle_rad': 0.29,
                'command_stale_timeout_sec': 0.5,
                'publish_rate_hz': 20.0,
            }],
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
