#ifndef MPC_NODE_HPP_
#define MPC_NODE_HPP_

#include <Eigen/Eigen>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Your MPC solver. NOTE: this header is not in front of me — the members and
// dimensions I reference on `mpc_` below (ugv.n, ugv.m, ugv.X0, etc.) are
// inferred from f_mpc_uncut.cpp's usage, not confirmed from the header.
// See the integration notes in mpc_node.cpp before trusting this to compile.
#include "f_mpc_uncut.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MPCNode : public rclcpp::Node {
public:
    MPCNode();

private:
    // --- ROS2 I/O, matching pure_pursuit.cpp's topics/params exactly ---
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    void param_timer_callback();
    void publish_drive(double steering_angle, double speed);
    void visualize_predicted_trajectory(const Eigen::MatrixXf &X);

    // --- Parameters (same names/defaults as PurePursuit where applicable) ---
    std::string odom_topic;
    std::string drive_topic;
    std::string car_refFrame;
    std::string global_refFrame;
    std::string rviz_predicted_traj_topic;
    double control_period_s;  // maps to delta_t used by the MPC horizon

    // --- Current vehicle state, filled from Odometry each callback ---
    double x_car_world = 0.0;
    double y_car_world = 0.0;
    double yaw_car_world = 0.0;
    double vx_car = 0.0;
    double vy_car = 0.0;
    double yaw_rate_car = 0.0;

    // --- ROS2 interfaces ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_predicted_traj_pub;
    rclcpp::TimerBase::SharedPtr param_timer_;

    // --- The actual solver ---
    F_MPC_UNCUT mpc_;

    // Warm-started state/control trajectories passed into fmpcsolve each tick.
    // Dimensions (n x horizon, m x horizon) are a GUESS pending the header —
    // see mpc_node.cpp.
    Eigen::MatrixXf X_traj_;
    Eigen::MatrixXf U_traj_;
    int segment_number_ = 0;
};

#endif  // MPC_NODE_HPP_
