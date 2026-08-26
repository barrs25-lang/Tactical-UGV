#include "mpc_node.hpp"

#include <cmath>

// ============================================================================
// INTEGRATION NOTES — READ BEFORE TRUSTING THIS FILE TO COMPILE AS-IS
// ============================================================================
// I do not have f_mpc_uncut.h, only f_mpc_uncut.cpp. Everything below that
// touches `mpc_` internals (dimensions, ugv.X0, fmpcsolve's exact contract)
// is inferred from how f_mpc_uncut.cpp uses those members internally, not
// confirmed from the header. Send me the header and I'll correct this in one
// pass instead of you finding out at compile/runtime.
//
// Specific things I could NOT confirm:
//   1. Whether `ugv`, `ugv.n`, `ugv.m`, `ugv.X0` are public. If they're
//      private, you'll need either a public setter (e.g. update_ugv_pose()
//      already exists and takes state from a `pose` array/member — I don't
//      know if `pose` itself is public/settable from outside the class) or
//      a small accessor added to the class.
//   2. The exact semantics of fmpcsolve(X, U, segment_number) — whether X/U
//      need to be pre-sized before the call, whether X's first column must
//      hold the current state (warm start) vs. being purely output, and
//      what `segment_number` selects (looks like a waypoint/goal segment
//      index based on find_new_waypoint/find_closest_waypoint, i.e. this
//      class expects to be fed a *trajectory* of goals, not a single
//      lookahead point like pure_pursuit — see note 4 below).
//   3. How U's rows map to actual control commands. get_f_k / get_theta_vf /
//      get_theta_vr suggest state includes vx, vy, psidot (consistent with
//      your bicycle-model adaptation), but I can't confirm which U row is
//      steering vs. drive force/acceleration.
//   4. How to feed it a goal/path. The original class uses `localPath` and
//      `find_closest_waypoint`/`find_new_waypoint` — there is no equivalent
//      of pure_pursuit's simple "load one CSV, pick a lookahead point" flow.
//      You will likely need to feed it waypoints from the same CSV
//      (waypoints_path) pure_pursuit uses, but through whatever public
//      method actually sets `localPath`/`goal` — not visible to me here.
//
// Two concrete bugs I found while reading f_mpc_uncut.cpp, independent of
// the ROS2 wiring:
//   A. get_f_k's declared signature has a DUPLICATE parameter name:
//        void F_MPC_UNCUT::get_f_k(float vx, float vy, float psidot,
//                                   float vx, float Fd, float* f)
//      That's `vx` twice — this will not compile as written. Almost
//      certainly a copy/paste typo (maybe the 4th param should be `vr` or
//      similar per get_theta_vr's naming). Fix in the source, not here.
//   B. F_MPC_UNCUT::start() installs its own SIGINT/SIGSEGV/SIGABRT handlers
//      and spins up pthreads with a blocking while loop. Do NOT call
//      mpc_.start() from inside this node — it will fight rclcpp's own
//      signal handling and block the executor thread forever. This file
//      calls fmpcsolve() directly per control tick instead, bypassing
//      start()'s threading model entirely. Confirm fmpcsolve doesn't itself
//      depend on state that only start()'s threads set up (e.g. traj_status
//      flags, mutex init via pthread_mutex_init in start()) — if it does,
//      you'll need to hoist that initialization into this node's
//      constructor instead.
// ============================================================================

MPCNode::MPCNode() : Node("mpc_node") {
    // Parameters — names/defaults matched to pure_pursuit.cpp so you can
    // launch both against the same sim config without renaming anything.
    this->declare_parameter("odom_topic", "/ego_racecar/odom");
    this->declare_parameter("car_refFrame", "ego_racecar/base_link");
    this->declare_parameter("drive_topic", "/drive");
    this->declare_parameter("global_refFrame", "map");
    this->declare_parameter("rviz_predicted_traj_topic", "/mpc_predicted_traj");
    this->declare_parameter("control_period_s", 0.1);  // should match delta_t in your param file

    odom_topic = this->get_parameter("odom_topic").as_string();
    car_refFrame = this->get_parameter("car_refFrame").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();
    global_refFrame = this->get_parameter("global_refFrame").as_string();
    rviz_predicted_traj_topic = this->get_parameter("rviz_predicted_traj_topic").as_string();
    control_period_s = this->get_parameter("control_period_s").as_double();

    // Same subscribe/publish pattern as PurePursuit: state comes in via
    // odom, control goes out via AckermannDriveStamped on /drive.
    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 25, std::bind(&MPCNode::odom_callback, this, _1));

    publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
    vis_predicted_traj_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_predicted_traj_topic, 10);

    // Mirrors PurePursuit's periodic parameter refresh (its timer_callback).
    param_timer_ = this->create_wall_timer(2000ms, std::bind(&MPCNode::param_timer_callback, this));

    // --- ASSUMPTION: ugv.n / ugv.m are public and set by F_MPC_UNCUT's
    // constructor after reading the param file. If this doesn't compile,
    // that's exactly the header-dependent piece flagged above. ---
    // int horizon = /* T from your param file, e.g. mpc_.T or similar */ 20;
    // X_traj_ = Eigen::MatrixXf::Zero(mpc_.ugv.n, horizon + 1);
    // U_traj_ = Eigen::MatrixXf::Zero(mpc_.ugv.m, horizon);

    RCLCPP_INFO(this->get_logger(), "MPC node has been launched");
}

void MPCNode::param_timer_callback() {
    control_period_s = this->get_parameter("control_period_s").as_double();
}

void MPCNode::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) {
    // Position, directly from Odometry — no tf lookup needed here since
    // (unlike pure_pursuit) we want world-frame state for the MPC, not a
    // waypoint transformed into the car frame.
    x_car_world = odom_msg->pose.pose.position.x;
    y_car_world = odom_msg->pose.pose.position.y;

    // Yaw from quaternion (standard extraction, avoids pulling in tf2
    // conversion headers for one scalar).
    const auto &q = odom_msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw_car_world = std::atan2(siny_cosp, cosy_cosp);

    // Body-frame velocities — this is where get_theta_vf/get_theta_vr in
    // f_mpc_uncut.cpp expect vx, vy, psidot, so pull them straight from the
    // twist (already body-frame in nav_msgs/Odometry per the message spec).
    vx_car = odom_msg->twist.twist.linear.x;
    vy_car = odom_msg->twist.twist.linear.y;
    yaw_rate_car = odom_msg->twist.twist.angular.z;

    // --- ASSUMPTION: state ordering [x, y, yaw, vx, vy, psidot] into
    // ugv.X0 / bomt.eX0 via whatever public setter exists. Placeholder
    // shown; adjust once the header confirms the real member/order. ---
    //
    // mpc_.ugv.X0(0) = x_car_world;
    // mpc_.ugv.X0(1) = y_car_world;
    // mpc_.ugv.X0(2) = yaw_car_world;
    // mpc_.ugv.X0(3) = vx_car;
    // mpc_.ugv.X0(4) = vy_car;
    // mpc_.ugv.X0(5) = yaw_rate_car;
    //
    // bool success = mpc_.fmpcsolve(&X_traj_, &U_traj_, segment_number_);
    // if (!success) {
    //     RCLCPP_WARN(this->get_logger(), "fmpcsolve failed to converge this tick");
    //     return;  // hold last command rather than publish garbage
    // }
    //
    // --- ASSUMPTION: U_traj_ row 0 = steering, row 1 = speed/accel command.
    // Confirm against your bicycle-model adaptation before trusting this. ---
    // double steering_angle = U_traj_(0, 0);
    // double speed_cmd = U_traj_(1, 0);
    // publish_drive(steering_angle, speed_cmd);
    // visualize_predicted_trajectory(X_traj_);

    RCLCPP_WARN_ONCE(this->get_logger(),
        "odom_callback is receiving state but the fmpcsolve() call is commented "
        "out pending confirmation of F_MPC_UNCUT's header. See integration "
        "notes at the top of mpc_node.cpp.");
}

void MPCNode::publish_drive(double steering_angle, double speed) {
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = speed;
    publisher_drive->publish(drive_msg);
}

void MPCNode::visualize_predicted_trajectory(const Eigen::MatrixXf &X) {
    // Publishes the MPC's predicted state trajectory as a line strip in
    // rviz, same visualization pattern PurePursuit uses for its lookahead
    // point, just extended to a full horizon. Assumes X's rows 0,1 are x,y —
    // same caveat as above.
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = global_refFrame;
    marker.header.stamp = rclcpp::Clock().now();
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.g = 1.0;

    for (int i = 0; i < X.cols(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = X(0, i);
        p.y = X(1, i);
        p.z = 0.0;
        marker.points.push_back(p);
    }
    vis_predicted_traj_pub->publish(marker);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<MPCNode>();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
