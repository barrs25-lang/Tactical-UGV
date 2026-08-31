#ifndef TACTICAL_UGV_AUTONOMOUS_STACK_LEGACY_BRIDGE_NODE_HPP
#define TACTICAL_UGV_AUTONOMOUS_STACK_LEGACY_BRIDGE_NODE_HPP

// LegacyBridgeNode replaces comm_server for exactly one legacy client (goal_generation,
// path_planner, or trajectory_planner): it reads that client's own socket_parameters_*.txt
// file, and for every socket the file marks active, either
//   - listens/accepts/forwards a ROS2 topic down the socket ("*_from_server", i.e. data the
//     legacy binary receives), or
//   - listens/accepts/receives off the socket and republishes it as a ROS2 topic
//     ("*_to_server", i.e. data the legacy binary sends).
// The legacy binary itself is never modified -- it still connects out to 127.0.0.1 exactly as
// it always has; this node just plays the server role on the other end. All 12 legacy socket
// roles are implemented (not only the ones the three known packages currently use), plus the
// 13th package-specific "control_to_server" socket trajectory_planner uses to send its planned
// control sequence (F_x, delta_f), so any future producer/consumer of
// map/pose/goal/path/constraints/trajectory/control can be wired in later without further
// changes here -- each of the three node executables (one built from src/goal_generation/src/,
// src/path_planner/src/, src/trajectory_planner/src/) only ever activates the sockets its own
// package's config file marks active, so it only ever publishes and subscribes to the topics
// that package actually produced/consumed under comm_server.

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tactical_ugv_autonomous_stack/msg/constraint_set.hpp"
#include "tactical_ugv_autonomous_stack/msg/control_sequence.hpp"
#include "tactical_ugv_autonomous_stack/msg/legacy_pose.hpp"
#include "tactical_ugv_autonomous_stack/msg/planner_trajectory.hpp"
#include "tactical_ugv_autonomous_stack/msg/voxel_map.hpp"

#include "tactical_ugv_autonomous_stack/latest_message_box.hpp"
#include "tactical_ugv_autonomous_stack/legacy_socket_server.hpp"
#include "tactical_ugv_autonomous_stack/socket_params.hpp"

namespace tactical_ugv_autonomous_stack
{

class LegacyBridgeNode : public rclcpp::Node
{
public:
	// num_sockets: how many (port, name, active) blocks default_config_path has -- 12 for
	// goal_generation/path_planner, 13 for trajectory_planner (which has the extra
	// "control_to_server" socket).
	explicit LegacyBridgeNode(
		const std::string & node_name, const std::string & default_config_path,
		int num_sockets = kNumSockets);
	~LegacyBridgeNode() override;

private:
	// Declares (with the given defaults) and reads back the topic-name / config-path parameters.
	void declare_and_read_parameters(const std::string & default_config_path);

	// Starts one background thread per socket the config file marks active.
	void start_active_sockets();

	// "*_from_server" loops: block on the LatestMessageBox for a new ROS2 message, encode it
	// into the legacy wire format, and send it once a legacy client has connected.
	void run_map_from_server(SocketDesc desc);
	void run_pose_from_server(SocketDesc desc);
	void run_goal_from_server(SocketDesc desc);
	void run_path_from_server(SocketDesc desc);
	void run_constraint_from_server(SocketDesc desc);
	void run_trajectory_from_server(SocketDesc desc);

	// "*_to_server" loops: accept a legacy client connection, then repeatedly receive a
	// fixed-size buffer, decode it, and publish the corresponding ROS2 message.
	void run_map_to_server(SocketDesc desc);
	void run_pose_to_server(SocketDesc desc);
	void run_goal_to_server(SocketDesc desc);
	void run_path_to_server(SocketDesc desc);
	void run_constraint_to_server(SocketDesc desc);
	void run_trajectory_to_server(SocketDesc desc);

	// index 12, package-specific: trajectory_planner's control-sequence output. There is no
	// "control_from_server" counterpart -- nothing currently sends a control sequence in.
	void run_control_to_server(SocketDesc desc);

	std::vector<SocketDesc> sockets_;

	// Topic names (ROS2 parameters, default "/ugv/<type>" so producer/consumer bridge nodes
	// naturally connect through the ROS2 graph the same way comm_server used to relay them).
	std::string map_topic_;
	std::string pose_topic_;
	std::string goal_topic_;
	std::string path_topic_;
	std::string constraint_topic_;
	std::string trajectory_topic_;
	std::string control_topic_;

	// Subscriptions feeding the "*_from_server" sender threads.
	rclcpp::Subscription<tactical_ugv_autonomous_stack::msg::VoxelMap>::SharedPtr map_sub_;
	rclcpp::Subscription<tactical_ugv_autonomous_stack::msg::LegacyPose>::SharedPtr pose_sub_;
	rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
	rclcpp::Subscription<tactical_ugv_autonomous_stack::msg::ConstraintSet>::SharedPtr constraint_sub_;
	rclcpp::Subscription<tactical_ugv_autonomous_stack::msg::PlannerTrajectory>::SharedPtr trajectory_sub_;

	LatestMessageBox<tactical_ugv_autonomous_stack::msg::VoxelMap> map_box_;
	LatestMessageBox<tactical_ugv_autonomous_stack::msg::LegacyPose> pose_box_;
	LatestMessageBox<geometry_msgs::msg::PointStamped> goal_box_;
	LatestMessageBox<nav_msgs::msg::Path> path_box_;
	LatestMessageBox<tactical_ugv_autonomous_stack::msg::ConstraintSet> constraint_box_;
	LatestMessageBox<tactical_ugv_autonomous_stack::msg::PlannerTrajectory> trajectory_box_;

	// Publishers fed by the "*_to_server" receiver threads.
	rclcpp::Publisher<tactical_ugv_autonomous_stack::msg::VoxelMap>::SharedPtr map_pub_;
	rclcpp::Publisher<tactical_ugv_autonomous_stack::msg::LegacyPose>::SharedPtr pose_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
	rclcpp::Publisher<tactical_ugv_autonomous_stack::msg::ConstraintSet>::SharedPtr constraint_pub_;
	rclcpp::Publisher<tactical_ugv_autonomous_stack::msg::PlannerTrajectory>::SharedPtr trajectory_pub_;
	rclcpp::Publisher<tactical_ugv_autonomous_stack::msg::ControlSequence>::SharedPtr control_pub_;

	std::vector<std::thread> threads_;
	std::atomic<bool> stop_{false};
};

}  // namespace tactical_ugv_autonomous_stack

#endif  // TACTICAL_UGV_AUTONOMOUS_STACK_LEGACY_BRIDGE_NODE_HPP
