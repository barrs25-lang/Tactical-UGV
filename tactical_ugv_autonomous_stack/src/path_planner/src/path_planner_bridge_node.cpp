// Replaces comm_server for the legacy path_planner binary: exposes its map/pose/goal inputs
// and path output as ROS2 topics. path_planner itself is not modified -- it still connects out
// to 127.0.0.1 on the ports in its own socket_parameters_path_planner.txt; this node plays the
// server role on the other end (see
// tactical_ugv_autonomous_stack/legacy_bridge_node.hpp for how).

#include "tactical_ugv_autonomous_stack/legacy_bridge_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);

	const std::string default_config =
		ament_index_cpp::get_package_share_directory("tactical_ugv_autonomous_stack") +
		"/config/socket_parameters_path_planner.txt";

	auto node = std::make_shared<tactical_ugv_autonomous_stack::LegacyBridgeNode>(
		"path_planner_bridge", default_config);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
