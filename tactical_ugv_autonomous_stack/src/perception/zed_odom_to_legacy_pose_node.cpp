// Translates the ZED wrapper's odometry into the LegacyPose the four legacy-bridge nodes
// expect on /ugv/pose (see src/legacy_bridge_node.cpp's pose_topic default and
// msg/LegacyPose.msg's 18-float layout, a leftover quadrotor state vector: position, velocity,
// acceleration, jerk, roll/pitch/yaw, and body angular rates).
//
// Field usage audit across the four legacy consumers (goal_generation.cpp, LPAstar.cpp,
// constraint.cpp, f_mpc_communication.cpp/f_mpc_trajectory.cpp/f_mpc_feedback_linearization.cpp):
// x,y,z (position), phi,theta,psi (full Euler orientation), dx,dy,dz (velocity), and omega3
// (yaw rate) are all read by at least one consumer. ddx..dddz (accel/jerk) and omega1/omega2
// (roll/pitch rate) are never read by anything -- dead quadrotor-era fields kept only because
// the wire format is a fixed 18-float array -- so they are zero-filled below intentionally.
//
// Velocity source: the installed ZED ROS2 wrapper's publishOdom() (zed_camera_component_main.cpp)
// only ever fills pose, never twist -- /zed/zed_node/odom's twist is permanently zero on this
// wrapper build, regardless of configuration. So dx,dy,dz,omega3 are computed here by
// finite-differencing consecutive position/yaw samples using their header timestamps, rather
// than read from twist. Since x,y,z are already world-frame (odom's own pose frame), the
// differenced dx,dy,dz are automatically world-frame too -- no body-to-world rotation is needed
// (unlike a sensor-reported body-frame twist would require, since constraint_generation and
// fmpc_uncut apply the same world-frame axis remap to both position and velocity: see
// constraint.cpp's X0[0]=y,X0[1]=-x and V0[0]=dy,V0[1]=-dx, and f_mpc_feedback_linearization.cpp's
// explicit body-frame recovery vx=xdot*cos(psi)+ydot*sin(psi), which only makes sense if xdot,ydot
// are already world-frame).

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tactical_ugv_autonomous_stack/msg/legacy_pose.hpp"

class ZedOdomToLegacyPose : public rclcpp::Node
{
public:
	ZedOdomToLegacyPose()
	: Node("zed_odom_to_legacy_pose")
	{
		this->declare_parameter<std::string>("odom_topic", "/zed/zed_node/odom");
		this->declare_parameter<std::string>("pose_topic", "/ugv/pose");

		const std::string odom_topic = this->get_parameter("odom_topic").as_string();
		const std::string pose_topic = this->get_parameter("pose_topic").as_string();

		pose_pub_ = this->create_publisher<tactical_ugv_autonomous_stack::msg::LegacyPose>(
			pose_topic, rclcpp::QoS(10));

		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			odom_topic, rclcpp::QoS(10),
			[this](const nav_msgs::msg::Odometry::SharedPtr msg) {on_odom(msg);});

		RCLCPP_INFO(get_logger(), "Subscribed to [%s], publishing LegacyPose on [%s]",
			odom_topic.c_str(), pose_topic.c_str());
	}

private:
	void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		tactical_ugv_autonomous_stack::msg::LegacyPose out;
		out.header = msg->header;

		out.x = msg->pose.pose.position.x;
		out.y = msg->pose.pose.position.y;
		out.z = msg->pose.pose.position.z;

		const auto & q = msg->pose.pose.orientation;
		const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
		const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
		out.phi = std::atan2(sinr_cosp, cosr_cosp);

		double sinp = 2.0 * (q.w * q.y - q.z * q.x);
		sinp = std::clamp(sinp, -1.0, 1.0);
		out.theta = std::asin(sinp);

		const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
		const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
		out.psi = std::atan2(siny_cosp, cosy_cosp);

		const rclcpp::Time stamp(msg->header.stamp);
		if (prev_.has_value()) {
			const double dt = (stamp - prev_->stamp).seconds();
			if (dt > 0.0) {
				out.dx = (out.x - prev_->x) / dt;
				out.dy = (out.y - prev_->y) / dt;
				out.dz = (out.z - prev_->z) / dt;
				// Shortest-path angle difference so crossing the +-pi wrap doesn't spike omega3.
				const double dpsi = std::atan2(
					std::sin(out.psi - prev_->psi), std::cos(out.psi - prev_->psi));
				out.omega3 = dpsi / dt;
			}
		}
		prev_ = {stamp, out.x, out.y, out.z, out.psi};

		out.ddx = out.ddy = out.ddz = 0.0;
		out.dddx = out.dddy = out.dddz = 0.0;
		out.omega1 = out.omega2 = 0.0;

		pose_pub_->publish(out);
	}

	struct PrevSample
	{
		rclcpp::Time stamp;
		double x, y, z, psi;
	};
	std::optional<PrevSample> prev_;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Publisher<tactical_ugv_autonomous_stack::msg::LegacyPose>::SharedPtr pose_pub_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ZedOdomToLegacyPose>());
	rclcpp::shutdown();
	return 0;
}
