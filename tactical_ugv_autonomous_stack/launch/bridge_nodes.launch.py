"""Launches the full stack: the three comm_server-replacement bridge nodes, plus the three
legacy planner binaries they talk to.

Data flow: goal_generation_bridge publishes /ugv/goal, which path_planner_bridge consumes and
turns into /ugv/path, which trajectory_planner_bridge consumes and turns into
/ugv/trajectory (state) and /ugv/control_sequence (F_x, delta_f). /ugv/map, /ugv/pose, and
/ugv/constraints have no publisher yet -- they're meant to come from a future perception/
localization/constraint-generation node (e.g. a ZED camera pipeline); until one exists, the
corresponding legacy binaries will simply block waiting for that first message, exactly as
they already block waiting for a TCP connection.

The legacy binaries (goal_generation, path_planner, fmpc_uncut) are built by each package's
own plain CMakeLists.txt under tactical_ugv_autonomous_stack/src/<package>/build/, not by this
ament_cmake package's colcon build -- there is no ROS-standard way to locate them from an
installed package, so their absolute paths in this checkout are hardcoded below. Each one
busy-loops connect() until its bridge node is listening, so start order between a bridge and
its binary doesn't matter -- but you do need to have actually built all three first (cmake ..
&& make in each package's own build/ directory) or the corresponding ExecuteProcess will just
fail to find its executable.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# Absolute path to this checkout's tactical_ugv_autonomous_stack/src -- see module docstring
# for why this can't be resolved from the installed package instead.
_STACK_SRC = '/home/bshla/UGV_Thesis/Tactical-UGV/tactical_ugv_autonomous_stack/src'


def generate_launch_description():
    return LaunchDescription([
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
    ])
