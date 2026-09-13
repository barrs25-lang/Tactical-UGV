// Translates fmpc_uncut's planned control sequence (/ugv/control_sequence, [vx, delta_f] per
// horizon step -- see msg/ControlSequence.msg and f_mpc_feedback_linearization.cpp's v_k for
// where those values actually come from) into the f1tenth stack's actuation command on /drive
// (ackermann_msgs/AckermannDriveStamped), which /ackermann_mux arbitrates (priority 10) against
// the joystick's /teleop (priority 100, the vehicle's only safety override -- there is no
// separate e-stop) before it reaches the VESC.
//
// Only data[0] (vx) and data[1] (delta_f) are used -- segment 0 / horizon-step 0 of the flat
// layout data[j*(T*m)+i*m+k], i.e. the receding-horizon "apply now" command. num_segments/
// horizon/control_dim on the message are always 0 (see legacy_bridge_node.cpp), so a stride
// can't be computed from the message itself; m=2 is hardcoded here to match System_params.txt.
//
// Two safety mechanisms live in this node, both independent of the mux's own 0.2s timeout:
//   - max_speed_mps / max_steering_angle_rad clamp every command before publishing. vx has NO
//     bound anywhere else in the pipeline (unlike delta_f, which the solver bounds at
//     delta_f_max=0.5 rad -- looser than this vehicle's real ~0.29 rad servo saturation, so that
//     bound can't be trusted here either).
//   - A wall timer republishes the latest clamped command at publish_rate_hz (well above the
//     mux's 5Hz/0.2s requirement, since control_sequence's own update rate depends on how fast
//     fmpc_uncut replans and isn't guaranteed to keep the mux's input fresh on its own), and
//     falls back to a safe-stop (speed=0, steering_angle=0) if no new control_sequence message
//     has arrived within command_stale_timeout_sec.
//
// Sign-convention note: it is not yet confirmed whether this vehicle's bicycle-model delta_f
// convention matches AckermannDriveStamped's documented "positive = left". Verify on the bench
// (wheels off the ground) before relying on this node to steer correctly; if backwards, negate
// delta_f at the single assignment point below -- do not change the solver or the message.

#include <algorithm>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "tactical_ugv_autonomous_stack/msg/control_sequence.hpp"

namespace
{
constexpr size_t kControlDim = 2;  // [vx, delta_f] -- must match quadrotor.m in System_params.txt
}  // namespace

class ControlSequenceToAckermann : public rclcpp::Node
{
public:
	ControlSequenceToAckermann()
	: Node("control_sequence_to_ackermann")
	{
		this->declare_parameter<std::string>("control_sequence_topic", "/ugv/control_sequence");
		this->declare_parameter<std::string>("drive_topic", "/drive");
		this->declare_parameter<double>("max_speed_mps", 1.0);
		this->declare_parameter<double>("max_steering_angle_rad", 0.29);
		this->declare_parameter<double>("command_stale_timeout_sec", 0.5);
		this->declare_parameter<double>("publish_rate_hz", 20.0);

		const std::string control_sequence_topic = this->get_parameter("control_sequence_topic").as_string();
		const std::string drive_topic = this->get_parameter("drive_topic").as_string();
		max_speed_mps_ = this->get_parameter("max_speed_mps").as_double();
		max_steering_angle_rad_ = this->get_parameter("max_steering_angle_rad").as_double();
		command_stale_timeout_sec_ = this->get_parameter("command_stale_timeout_sec").as_double();
		const double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();

		drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
			drive_topic, rclcpp::QoS(1));

		control_sub_ = this->create_subscription<tactical_ugv_autonomous_stack::msg::ControlSequence>(
			control_sequence_topic, rclcpp::QoS(1),
			[this](const tactical_ugv_autonomous_stack::msg::ControlSequence::SharedPtr msg) {on_control(msg);});

		const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
		timer_ = this->create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(period),
			[this]() {on_timer();});

		RCLCPP_INFO(get_logger(),
			"Subscribed to [%s], publishing AckermannDriveStamped on [%s] at %.1f Hz "
			"(max_speed_mps=%.2f, max_steering_angle_rad=%.2f, stale_timeout=%.2fs)",
			control_sequence_topic.c_str(), drive_topic.c_str(), publish_rate_hz,
			max_speed_mps_, max_steering_angle_rad_, command_stale_timeout_sec_);
	}

private:
	void on_control(const tactical_ugv_autonomous_stack::msg::ControlSequence::SharedPtr msg)
	{
		if (msg->data.size() < kControlDim) {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
				"Received ControlSequence with data.size()=%zu, need at least %zu -- ignoring",
				msg->data.size(), kControlDim);
			return;
		}

		const double raw_vx = msg->data[0];
		const double raw_delta_f = msg->data[1];

		std::lock_guard<std::mutex> lock(mutex_);
		latest_speed_ = std::clamp(raw_vx, -max_speed_mps_, max_speed_mps_);
		latest_steering_angle_ = std::clamp(raw_delta_f, -max_steering_angle_rad_, max_steering_angle_rad_);
		last_msg_time_ = this->now();
		have_command_ = true;
	}

	void on_timer()
	{
		double speed = 0.0;
		double steering_angle = 0.0;

		{
			std::lock_guard<std::mutex> lock(mutex_);
			const bool fresh = have_command_ &&
				(this->now() - last_msg_time_).seconds() <= command_stale_timeout_sec_;
			if (fresh) {
				speed = latest_speed_;
				steering_angle = latest_steering_angle_;
			}
			// else: stale or never received -- publish the safe-stop zeroed above.
		}

		ackermann_msgs::msg::AckermannDriveStamped out;
		out.header.stamp = this->now();
		out.drive.speed = static_cast<float>(speed);
		out.drive.steering_angle = static_cast<float>(steering_angle);
		drive_pub_->publish(out);
	}

	double max_speed_mps_;
	double max_steering_angle_rad_;
	double command_stale_timeout_sec_;

	std::mutex mutex_;
	double latest_speed_{0.0};
	double latest_steering_angle_{0.0};
	rclcpp::Time last_msg_time_;
	bool have_command_{false};

	rclcpp::Subscription<tactical_ugv_autonomous_stack::msg::ControlSequence>::SharedPtr control_sub_;
	rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ControlSequenceToAckermann>());
	rclcpp::shutdown();
	return 0;
}
