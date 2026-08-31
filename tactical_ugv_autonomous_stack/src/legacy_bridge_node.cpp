#include "tactical_ugv_autonomous_stack/legacy_bridge_node.hpp"
#include "tactical_ugv_autonomous_stack/legacy_csv_codec.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace tactical_ugv_autonomous_stack
{

namespace
{
// Legacy fixed buffer sizes -- both the legacy CLIENT and the original comm_server hardcode
// these exact sizes on every send()/recv() for a given stream (see
// trajectory_planner/comm_server/src/my_server.cpp and
// trajectory_planner/trajectory_planner/src/f_mpc_uncut/f_mpc_communication.cpp). They must
// match exactly or a MSG_WAITALL recv() on either end will block forever / misframe the
// stream that follows.
constexpr size_t kMapBufferSize = 300000;
constexpr size_t kPoseBufferSize = 1000;
constexpr size_t kGoalBufferSize = 100;
constexpr size_t kPathBufferSize = 5000;
constexpr size_t kConstraintBufferSize = 5000;
constexpr size_t kTrajectoryBufferSize = 6000;
constexpr size_t kControlBufferSize = 6000;
}  // namespace

LegacyBridgeNode::LegacyBridgeNode(
	const std::string & node_name, const std::string & default_config_path, int num_sockets)
: rclcpp::Node(node_name)
{
	declare_and_read_parameters(default_config_path);

	const std::string config_path = this->get_parameter("socket_params_file").as_string();
	sockets_ = load_socket_params(config_path, num_sockets);

	RCLCPP_INFO(get_logger(), "Loaded socket parameters from %s", config_path.c_str());

	start_active_sockets();
}

LegacyBridgeNode::~LegacyBridgeNode()
{
	stop_.store(true);
	map_box_.notify_stop();
	pose_box_.notify_stop();
	goal_box_.notify_stop();
	path_box_.notify_stop();
	constraint_box_.notify_stop();
	trajectory_box_.notify_stop();

	// The "*_to_server" threads may be blocked inside a blocking accept()/recv() syscall with
	// no portable, low-effort way to interrupt them; detaching (rather than joining) lets the
	// process exit cleanly, matching the legacy comm_server's own shutdown, which just called
	// exit(0) without unwinding its per-socket accept loops either.
	for (auto & t : threads_) {
		if (t.joinable()) {
			t.detach();
		}
	}
}

void LegacyBridgeNode::declare_and_read_parameters(const std::string & default_config_path)
{
	this->declare_parameter<std::string>("socket_params_file", default_config_path);
	this->declare_parameter<std::string>("map_topic", "/ugv/map");
	this->declare_parameter<std::string>("pose_topic", "/ugv/pose");
	this->declare_parameter<std::string>("goal_topic", "/ugv/goal");
	this->declare_parameter<std::string>("path_topic", "/ugv/path");
	this->declare_parameter<std::string>("constraint_topic", "/ugv/constraints");
	this->declare_parameter<std::string>("trajectory_topic", "/ugv/trajectory");
	this->declare_parameter<std::string>("control_topic", "/ugv/control_sequence");

	map_topic_ = this->get_parameter("map_topic").as_string();
	pose_topic_ = this->get_parameter("pose_topic").as_string();
	goal_topic_ = this->get_parameter("goal_topic").as_string();
	path_topic_ = this->get_parameter("path_topic").as_string();
	constraint_topic_ = this->get_parameter("constraint_topic").as_string();
	trajectory_topic_ = this->get_parameter("trajectory_topic").as_string();
	control_topic_ = this->get_parameter("control_topic").as_string();
}

void LegacyBridgeNode::start_active_sockets()
{
	// Wraps a run_* member function so a bind()/accept() failure on one socket (e.g. the port
	// is already in use) logs an error and only takes down that one thread, rather than
	// propagating out of std::thread and calling std::terminate() on the whole node.
	auto guarded = [this](void (LegacyBridgeNode::*fn)(SocketDesc), SocketDesc desc) {
			try {
				(this->*fn)(desc);
			} catch (const std::exception & e) {
				RCLCPP_ERROR(get_logger(), "[%s] fatal error: %s", desc.name.c_str(), e.what());
			}
		};

	// index 0/1: map
	if (sockets_[0].active) {
		map_sub_ = this->create_subscription<tactical_ugv_autonomous_stack::msg::VoxelMap>(
			map_topic_, rclcpp::QoS(1),
			[this](const tactical_ugv_autonomous_stack::msg::VoxelMap::SharedPtr msg) {map_box_.set(*msg);});
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_map_from_server, sockets_[0]);
	}
	if (sockets_[1].active) {
		map_pub_ = this->create_publisher<tactical_ugv_autonomous_stack::msg::VoxelMap>(map_topic_, rclcpp::QoS(1));
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_map_to_server, sockets_[1]);
	}

	// index 2/3: pose
	if (sockets_[2].active) {
		pose_sub_ = this->create_subscription<tactical_ugv_autonomous_stack::msg::LegacyPose>(
			pose_topic_, rclcpp::QoS(10),
			[this](const tactical_ugv_autonomous_stack::msg::LegacyPose::SharedPtr msg) {pose_box_.set(*msg);});
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_pose_from_server, sockets_[2]);
	}
	if (sockets_[3].active) {
		pose_pub_ = this->create_publisher<tactical_ugv_autonomous_stack::msg::LegacyPose>(pose_topic_, rclcpp::QoS(10));
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_pose_to_server, sockets_[3]);
	}

	// index 4/5: goal
	if (sockets_[4].active) {
		goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
			goal_topic_, rclcpp::QoS(10),
			[this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {goal_box_.set(*msg);});
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_goal_from_server, sockets_[4]);
	}
	if (sockets_[5].active) {
		goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(goal_topic_, rclcpp::QoS(10));
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_goal_to_server, sockets_[5]);
	}

	// index 6/7: path
	if (sockets_[6].active) {
		path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
			path_topic_, rclcpp::QoS(10),
			[this](const nav_msgs::msg::Path::SharedPtr msg) {path_box_.set(*msg);});
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_path_from_server, sockets_[6]);
	}
	if (sockets_[7].active) {
		path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(10));
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_path_to_server, sockets_[7]);
	}

	// index 8/9: constraints
	if (sockets_[8].active) {
		constraint_sub_ = this->create_subscription<tactical_ugv_autonomous_stack::msg::ConstraintSet>(
			constraint_topic_, rclcpp::QoS(10),
			[this](const tactical_ugv_autonomous_stack::msg::ConstraintSet::SharedPtr msg) {constraint_box_.set(*msg);});
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_constraint_from_server, sockets_[8]);
	}
	if (sockets_[9].active) {
		constraint_pub_ = this->create_publisher<tactical_ugv_autonomous_stack::msg::ConstraintSet>(
			constraint_topic_, rclcpp::QoS(10));
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_constraint_to_server, sockets_[9]);
	}

	// index 10/11: trajectory
	if (sockets_[10].active) {
		trajectory_sub_ = this->create_subscription<tactical_ugv_autonomous_stack::msg::PlannerTrajectory>(
			trajectory_topic_, rclcpp::QoS(10),
			[this](const tactical_ugv_autonomous_stack::msg::PlannerTrajectory::SharedPtr msg) {trajectory_box_.set(*msg);});
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_trajectory_from_server, sockets_[10]);
	}
	if (sockets_[11].active) {
		trajectory_pub_ = this->create_publisher<tactical_ugv_autonomous_stack::msg::PlannerTrajectory>(
			trajectory_topic_, rclcpp::QoS(10));
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_trajectory_to_server, sockets_[11]);
	}

	// index 12: control sequence (package-specific, trajectory_planner only). No
	// "control_from_server" counterpart exists.
	if (sockets_.size() > 12 && sockets_[12].active) {
		control_pub_ = this->create_publisher<tactical_ugv_autonomous_stack::msg::ControlSequence>(
			control_topic_, rclcpp::QoS(10));
		threads_.emplace_back(guarded, &LegacyBridgeNode::run_control_to_server, sockets_[12]);
	}
}

// ----------------------------------------------------------------------------------------
// "*_from_server": bridge subscribes to ROS2, forwards down the socket to the legacy client
// ----------------------------------------------------------------------------------------

void LegacyBridgeNode::run_map_from_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);
	server.accept_and_handshake();
	RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

	uint64_t last_seen = 0;
	tactical_ugv_autonomous_stack::msg::VoxelMap msg;
	while (!stop_.load()) {
		if (!map_box_.wait_for_update(last_seen, msg, stop_)) {
			break;
		}
		if (!server.send_exact(msg.data.data(), kMapBufferSize)) {
			RCLCPP_WARN(get_logger(), "[%s] send failed, waiting for reconnect", desc.name.c_str());
			server.close_client();
			server.accept_and_handshake();
		}
	}
}

void LegacyBridgeNode::run_pose_from_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);
	server.accept_and_handshake();
	RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

	uint64_t last_seen = 0;
	tactical_ugv_autonomous_stack::msg::LegacyPose msg;
	std::vector<char> buffer(kPoseBufferSize);
	while (!stop_.load()) {
		if (!pose_box_.wait_for_update(last_seen, msg, stop_)) {
			break;
		}
		std::vector<double> values = {
			msg.x, msg.y, msg.z, msg.dx, msg.dy, msg.dz, msg.ddx, msg.ddy, msg.ddz,
			msg.dddx, msg.dddy, msg.dddz, msg.phi, msg.theta, msg.psi,
			msg.omega1, msg.omega2, msg.omega3};
		if (!encode_legacy_csv('Q', values, buffer) || !server.send_exact(buffer.data(), buffer.size())) {
			RCLCPP_WARN(get_logger(), "[%s] send failed, waiting for reconnect", desc.name.c_str());
			server.close_client();
			server.accept_and_handshake();
		}
	}
}

void LegacyBridgeNode::run_goal_from_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);
	server.accept_and_handshake();
	RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

	uint64_t last_seen = 0;
	geometry_msgs::msg::PointStamped msg;
	std::vector<char> buffer(kGoalBufferSize);
	while (!stop_.load()) {
		if (!goal_box_.wait_for_update(last_seen, msg, stop_)) {
			break;
		}
		std::vector<double> values = {msg.point.x, msg.point.y, msg.point.z};
		if (!encode_legacy_csv('G', values, buffer) || !server.send_exact(buffer.data(), buffer.size())) {
			RCLCPP_WARN(get_logger(), "[%s] send failed, waiting for reconnect", desc.name.c_str());
			server.close_client();
			server.accept_and_handshake();
		}
	}
}

void LegacyBridgeNode::run_path_from_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);
	server.accept_and_handshake();
	RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

	uint64_t last_seen = 0;
	nav_msgs::msg::Path msg;
	std::vector<char> buffer(kPathBufferSize);
	while (!stop_.load()) {
		if (!path_box_.wait_for_update(last_seen, msg, stop_)) {
			break;
		}
		if (msg.poses.empty()) {
			continue;
		}
		std::vector<double> values;
		values.reserve(msg.poses.size() * 3);
		for (const auto & pose_stamped : msg.poses) {
			values.push_back(pose_stamped.pose.position.x);
			values.push_back(pose_stamped.pose.position.y);
			values.push_back(pose_stamped.pose.position.z);
		}
		if (!encode_legacy_csv('P', values, buffer) || !server.send_exact(buffer.data(), buffer.size())) {
			RCLCPP_WARN(get_logger(), "[%s] send failed, waiting for reconnect", desc.name.c_str());
			server.close_client();
			server.accept_and_handshake();
		}
	}
}

void LegacyBridgeNode::run_constraint_from_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);
	server.accept_and_handshake();
	RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

	uint64_t last_seen = 0;
	tactical_ugv_autonomous_stack::msg::ConstraintSet msg;
	std::vector<char> buffer(kConstraintBufferSize);
	while (!stop_.load()) {
		if (!constraint_box_.wait_for_update(last_seen, msg, stop_)) {
			break;
		}
		if (msg.halfspaces.empty()) {
			continue;
		}
		std::vector<double> values;
		values.reserve(msg.halfspaces.size() * 4);
		for (const auto & h : msg.halfspaces) {
			values.push_back(h.a);
			values.push_back(h.b);
			values.push_back(h.c);
			values.push_back(h.d);
		}
		if (!encode_legacy_csv('C', values, buffer) || !server.send_exact(buffer.data(), buffer.size())) {
			RCLCPP_WARN(get_logger(), "[%s] send failed, waiting for reconnect", desc.name.c_str());
			server.close_client();
			server.accept_and_handshake();
		}
	}
}

void LegacyBridgeNode::run_trajectory_from_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);
	server.accept_and_handshake();
	RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

	uint64_t last_seen = 0;
	tactical_ugv_autonomous_stack::msg::PlannerTrajectory msg;
	std::vector<char> buffer(kTrajectoryBufferSize);
	while (!stop_.load()) {
		if (!trajectory_box_.wait_for_update(last_seen, msg, stop_)) {
			break;
		}
		if (msg.data.empty()) {
			continue;
		}
		std::vector<double> values(msg.data.begin(), msg.data.end());
		if (!encode_legacy_csv('T', values, buffer) || !server.send_exact(buffer.data(), buffer.size())) {
			RCLCPP_WARN(get_logger(), "[%s] send failed, waiting for reconnect", desc.name.c_str());
			server.close_client();
			server.accept_and_handshake();
		}
	}
}

// ----------------------------------------------------------------------------------------
// "*_to_server": bridge receives off the socket from the legacy client, publishes to ROS2
// ----------------------------------------------------------------------------------------

void LegacyBridgeNode::run_map_to_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);

	std::vector<char> buffer(kMapBufferSize);
	while (!stop_.load()) {
		server.accept_and_handshake();
		RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

		while (!stop_.load()) {
			if (!server.recv_exact(buffer.data(), buffer.size())) {
				RCLCPP_WARN(get_logger(), "[%s] disconnected, waiting for reconnect", desc.name.c_str());
				break;
			}
			tactical_ugv_autonomous_stack::msg::VoxelMap msg;
			msg.header.stamp = this->now();
			msg.size_x = 100;
			msg.size_y = 30;
			msg.size_z = 100;
			msg.resolution = 0.2;
			std::copy(buffer.begin(), buffer.end(), msg.data.begin());
			map_pub_->publish(msg);
		}
		server.close_client();
	}
}

void LegacyBridgeNode::run_pose_to_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);

	std::vector<char> buffer(kPoseBufferSize);
	while (!stop_.load()) {
		server.accept_and_handshake();
		RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

		while (!stop_.load()) {
			if (!server.recv_exact(buffer.data(), buffer.size())) {
				RCLCPP_WARN(get_logger(), "[%s] disconnected, waiting for reconnect", desc.name.c_str());
				break;
			}
			std::vector<double> values;
			if (decode_legacy_csv(buffer, 'Q', values) && values.size() >= 18) {
				tactical_ugv_autonomous_stack::msg::LegacyPose msg;
				msg.header.stamp = this->now();
				msg.x = values[0]; msg.y = values[1]; msg.z = values[2];
				msg.dx = values[3]; msg.dy = values[4]; msg.dz = values[5];
				msg.ddx = values[6]; msg.ddy = values[7]; msg.ddz = values[8];
				msg.dddx = values[9]; msg.dddy = values[10]; msg.dddz = values[11];
				msg.phi = values[12]; msg.theta = values[13]; msg.psi = values[14];
				msg.omega1 = values[15]; msg.omega2 = values[16]; msg.omega3 = values[17];
				pose_pub_->publish(msg);
			}
		}
		server.close_client();
	}
}

void LegacyBridgeNode::run_goal_to_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);

	std::vector<char> buffer(kGoalBufferSize);
	while (!stop_.load()) {
		server.accept_and_handshake();
		RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

		while (!stop_.load()) {
			if (!server.recv_exact(buffer.data(), buffer.size())) {
				RCLCPP_WARN(get_logger(), "[%s] disconnected, waiting for reconnect", desc.name.c_str());
				break;
			}
			std::vector<double> values;
			if (decode_legacy_csv(buffer, 'G', values) && values.size() >= 3) {
				geometry_msgs::msg::PointStamped msg;
				msg.header.stamp = this->now();
				msg.point.x = values[0];
				msg.point.y = values[1];
				msg.point.z = values[2];
				goal_pub_->publish(msg);
			}
		}
		server.close_client();
	}
}

void LegacyBridgeNode::run_path_to_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);

	std::vector<char> buffer(kPathBufferSize);
	while (!stop_.load()) {
		server.accept_and_handshake();
		RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

		while (!stop_.load()) {
			if (!server.recv_exact(buffer.data(), buffer.size())) {
				RCLCPP_WARN(get_logger(), "[%s] disconnected, waiting for reconnect", desc.name.c_str());
				break;
			}
			std::vector<double> values;
			if (decode_legacy_csv(buffer, 'P', values) && values.size() >= 3 && values.size() % 3 == 0) {
				nav_msgs::msg::Path msg;
				msg.header.stamp = this->now();
				msg.header.frame_id = "map";
				for (size_t i = 0; i + 2 < values.size(); i += 3) {
					geometry_msgs::msg::PoseStamped pose_stamped;
					pose_stamped.header = msg.header;
					pose_stamped.pose.position.x = values[i];
					pose_stamped.pose.position.y = values[i + 1];
					pose_stamped.pose.position.z = values[i + 2];
					pose_stamped.pose.orientation.w = 1.0;
					msg.poses.push_back(pose_stamped);
				}
				path_pub_->publish(msg);
			}
		}
		server.close_client();
	}
}

void LegacyBridgeNode::run_constraint_to_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);

	std::vector<char> buffer(kConstraintBufferSize);
	while (!stop_.load()) {
		server.accept_and_handshake();
		RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

		while (!stop_.load()) {
			if (!server.recv_exact(buffer.data(), buffer.size())) {
				RCLCPP_WARN(get_logger(), "[%s] disconnected, waiting for reconnect", desc.name.c_str());
				break;
			}
			std::vector<double> values;
			if (decode_legacy_csv(buffer, 'C', values) && values.size() >= 4 && values.size() % 4 == 0) {
				tactical_ugv_autonomous_stack::msg::ConstraintSet msg;
				msg.header.stamp = this->now();
				for (size_t i = 0; i + 3 < values.size(); i += 4) {
					tactical_ugv_autonomous_stack::msg::Halfspace h;
					h.a = values[i]; h.b = values[i + 1]; h.c = values[i + 2]; h.d = values[i + 3];
					msg.halfspaces.push_back(h);
				}
				constraint_pub_->publish(msg);
			}
		}
		server.close_client();
	}
}

void LegacyBridgeNode::run_trajectory_to_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);

	std::vector<char> buffer(kTrajectoryBufferSize);
	while (!stop_.load()) {
		server.accept_and_handshake();
		RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

		while (!stop_.load()) {
			if (!server.recv_exact(buffer.data(), buffer.size())) {
				RCLCPP_WARN(get_logger(), "[%s] disconnected, waiting for reconnect", desc.name.c_str());
				break;
			}
			std::vector<double> values;
			if (decode_legacy_csv(buffer, 'T', values) && !values.empty()) {
				tactical_ugv_autonomous_stack::msg::PlannerTrajectory msg;
				msg.header.stamp = this->now();
				// The wire format carries no dimensions -- publish the flat data as-is; a
				// consumer that knows nu_X/T/n (mpc_params.nu_X, mpc_params.T, quadrotor.n)
				// can reshape it, or those can be added as node parameters later.
				msg.num_segments = 0;
				msg.horizon = 0;
				msg.state_dim = 0;
				msg.data.assign(values.begin(), values.end());
				trajectory_pub_->publish(msg);
			}
		}
		server.close_client();
	}
}

void LegacyBridgeNode::run_control_to_server(SocketDesc desc)
{
	LegacySocketServer server;
	server.bind_and_listen(desc.port, desc.name);
	RCLCPP_INFO(get_logger(), "[%s] listening on port %d", desc.name.c_str(), desc.port);

	std::vector<char> buffer(kControlBufferSize);
	while (!stop_.load()) {
		server.accept_and_handshake();
		RCLCPP_INFO(get_logger(), "[%s] legacy client connected", desc.name.c_str());

		while (!stop_.load()) {
			if (!server.recv_exact(buffer.data(), buffer.size())) {
				RCLCPP_WARN(get_logger(), "[%s] disconnected, waiting for reconnect", desc.name.c_str());
				break;
			}
			std::vector<double> values;
			if (decode_legacy_csv(buffer, 'U', values) && !values.empty()) {
				tactical_ugv_autonomous_stack::msg::ControlSequence msg;
				msg.header.stamp = this->now();
				// The wire format carries no dimensions -- publish the flat data as-is; a
				// consumer that knows nu_X/T/m (mpc_params.nu_X, mpc_params.T, quadrotor.m)
				// can reshape it, or those can be added as node parameters later.
				msg.num_segments = 0;
				msg.horizon = 0;
				msg.control_dim = 0;
				msg.data.assign(values.begin(), values.end());
				control_pub_->publish(msg);
			}
		}
		server.close_client();
	}
}

}  // namespace tactical_ugv_autonomous_stack
