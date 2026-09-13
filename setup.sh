#!/usr/bin/env bash
# setup.sh -- installs every dependency and builds every binary in this repo.
#
# Usage:
#   ./setup.sh              Install dependencies (apt + rosdep), then build everything.
#   ./setup.sh --skip-apt   Skip dependency installation (e.g. re-running after a code change,
#                           or building somewhere without sudo) and just build.
#
# What gets built:
#   1. The four legacy planner binaries, each a standalone plain-CMake package with its own
#      pre-existing build/ directory (not part of the ROS2/colcon build):
#        tactical_ugv_autonomous_stack/src/goal_generation/build/goal_generation
#        tactical_ugv_autonomous_stack/src/path_planner/build/path_planner
#        tactical_ugv_autonomous_stack/src/trajectory_planner/build/fmpc_uncut
#        tactical_ugv_autonomous_stack/src/constraint_generation/build/constraint_generation
#   2. The tactical_ugv_autonomous_stack ROS2 ament_cmake package (bridge nodes, custom
#      messages, perception translator nodes), via colcon.
#
# NOT covered by this script (pre-existing, external, assumed already set up separately):
#   - ROS2 Humble itself (this script assumes /opt/ros/humble already exists).
#   - The ZED camera SDK/ROS2 wrapper.
#   - The separate f1tenth_ws workspace (VESC driver, ackermann_mux, joystick teleop) that
#     /drive commands are published to -- see tactical_ugv_autonomous_stack/launch/
#     bridge_nodes.launch.py's module docstring for the full data-flow contract.
#
# Run from anywhere; paths below are resolved relative to this script's own location, so it
# works regardless of the machine or clone path (matching the convention already used by
# bridge_nodes.launch.py).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

SKIP_APT=0
for arg in "$@"; do
	case "$arg" in
		--skip-apt) SKIP_APT=1 ;;
		*)
			echo "Unknown argument: $arg" >&2
			echo "Usage: $0 [--skip-apt]" >&2
			exit 1
			;;
	esac
done

if [ ! -d /opt/ros/humble ]; then
	echo "ERROR: /opt/ros/humble not found. Install ROS2 Humble first --" \
		"this script only adds this repo's own extra dependencies on top of it." >&2
	exit 1
fi

if [ "$SKIP_APT" -eq 0 ]; then
	echo "==> Installing dependencies (requires sudo)..."
	export DEBIAN_FRONTEND=noninteractive

	sudo apt-get update

	# Plain system libraries needed by the four legacy CMake packages (goal_generation,
	# path_planner, trajectory_planner/fmpc_uncut, constraint_generation). Eigen is vendored
	# under each package's own include/Eigen/ (a full bundled copy checked into the repo), so
	# libeigen3-dev is deliberately NOT installed here -- none of the four builds use the
	# system one.
	sudo apt-get install -y \
		build-essential cmake \
		libncurses-dev \
		libx11-dev libxrandr-dev libxi-dev \
		libboost-dev \
		liblapack-dev libatlas-base-dev libmumps-seq-dev libsdpa-dev

	# octomap_server is a runtime launch-graph dependency (bridge_nodes.launch.py starts it as
	# package='octomap_server') but isn't a tactical_ugv_autonomous_stack package.xml <depend>,
	# so rosdep below won't pull it in on its own.
	sudo apt-get install -y ros-humble-octomap-server

	# Everything tactical_ugv_autonomous_stack/package.xml declares (rclcpp, octomap_msgs,
	# ackermann_msgs, etc.) -- resolved via rosdep so this stays correct if package.xml changes,
	# rather than hardcoding a duplicate apt list here.
	if ! command -v rosdep >/dev/null 2>&1; then
		sudo apt-get install -y python3-rosdep
	fi
	sudo rosdep init 2>/dev/null || true  # already-initialized on this machine is not an error
	rosdep update
	rosdep install --from-paths tactical_ugv_autonomous_stack --ignore-src -r -y
else
	echo "==> --skip-apt passed, skipping dependency installation."
fi

# ROS2's own setup.bash references variables it doesn't guarantee are set, which trips our
# `set -u` above -- relax it just for the source, then restore it.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

echo ""
echo "==> Building the four legacy planner binaries..."

LEGACY_PACKAGES=(
	tactical_ugv_autonomous_stack/src/goal_generation
	tactical_ugv_autonomous_stack/src/path_planner
	tactical_ugv_autonomous_stack/src/trajectory_planner
	tactical_ugv_autonomous_stack/src/constraint_generation
)

for pkg in "${LEGACY_PACKAGES[@]}"; do
	echo "--- $pkg ---"
	mkdir -p "$pkg/build"
	(
		cd "$pkg/build"
		cmake ..
		make -j"$(nproc)"
	)
done

echo ""
echo "==> Building tactical_ugv_autonomous_stack (colcon)..."
colcon build --packages-select tactical_ugv_autonomous_stack

echo ""
echo "==> Done. Binaries:"
echo "  tactical_ugv_autonomous_stack/src/goal_generation/build/goal_generation"
echo "  tactical_ugv_autonomous_stack/src/path_planner/build/path_planner"
echo "  tactical_ugv_autonomous_stack/src/trajectory_planner/build/fmpc_uncut"
echo "  tactical_ugv_autonomous_stack/src/constraint_generation/build/constraint_generation"
echo "  install/tactical_ugv_autonomous_stack/lib/tactical_ugv_autonomous_stack/*"
echo ""
echo "To run the full stack:"
echo "  source install/setup.bash"
echo "  ros2 launch tactical_ugv_autonomous_stack bridge_nodes.launch.py"
