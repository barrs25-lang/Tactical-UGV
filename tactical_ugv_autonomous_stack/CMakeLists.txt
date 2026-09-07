cmake_minimum_required(VERSION 3.8)
project(tactical_ugv_autonomous_stack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)

# Custom messages mirroring the legacy comm_server wire formats (pose, voxel map, collision
# constraints, planned trajectory). Goal and path reuse standard geometry_msgs/nav_msgs types.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegacyPose.msg"
  "msg/VoxelMap.msg"
  "msg/Halfspace.msg"
  "msg/ConstraintSet.msg"
  "msg/PlannerTrajectory.msg"
  "msg/ControlSequence.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)

# Shared bridge-node implementation (comm_server replacement logic), used by all three
# per-package node executables below. Static: it's purely internal to this package's own
# three executables, so there's no need to install it as a shared library on the runtime
# linker path.
add_library(legacy_bridge_core STATIC
  src/legacy_bridge_node.cpp
)
target_link_libraries(legacy_bridge_core "${cpp_typesupport_target}")
ament_target_dependencies(legacy_bridge_core
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# One node executable per legacy package, built from that package's own src/ directory
# (the legacy binaries themselves -- goal_generation, path_planner, fmpc_uncut,
# constraint_generation -- are built separately by each package's own existing, untouched
# CMakeLists.txt/build process and are not part of this ament_cmake build).
add_executable(goal_generation_bridge src/goal_generation/src/goal_generation_bridge_node.cpp)
add_executable(path_planner_bridge src/path_planner/src/path_planner_bridge_node.cpp)
add_executable(trajectory_planner_bridge src/trajectory_planner/src/trajectory_planner_bridge_node.cpp)
add_executable(constraint_generation_bridge src/constraint_generation/src/constraint_generation_bridge_node.cpp)

foreach(bridge_target goal_generation_bridge path_planner_bridge trajectory_planner_bridge constraint_generation_bridge)
  target_link_libraries(${bridge_target} legacy_bridge_core "${cpp_typesupport_target}")
  ament_target_dependencies(${bridge_target}
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    ament_index_cpp
  )
endforeach()

# Translator node: deserializes octomap_server's octomap_binary output and republishes it as a
# VoxelMap on /ugv/map in the fixed 100x30x100 @ 0.2m dense layout the legacy binaries expect
# (see include/tactical_ugv_autonomous_stack/octomap_to_voxelmap.md-equivalent notes in the
# source file itself for the exact index/byte-order contract). octomap is a plain CMake-exported
# package (not ament), hence the extra include/link variables below instead of
# ament_target_dependencies alone.
add_executable(octomap_to_voxelmap src/perception/octomap_to_voxelmap_node.cpp)
target_include_directories(octomap_to_voxelmap PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(octomap_to_voxelmap "${cpp_typesupport_target}" ${OCTOMAP_LIBRARIES})
ament_target_dependencies(octomap_to_voxelmap
  rclcpp
  std_msgs
  octomap_msgs
)

install(TARGETS
  goal_generation_bridge
  path_planner_bridge
  trajectory_planner_bridge
  constraint_generation_bridge
  octomap_to_voxelmap
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
