/**
 * balance_controller.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Cascade PID — inner balance loop + outer velocity loop.
 * See balance_controller.hpp for documentation.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "balance_controller.hpp"

#include <cmath>
#include <algorithm>

#include <std_msgs/msg/float64_multi_array.hpp>

namespace segway_control
{

namespace
{
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr g_pub_wheel_effort;

void publish_wheel_effort(double left_effort, double right_effort)
{
    if (!g_pub_wheel_effort) {
        return;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {left_effort, right_effort};
    g_pub_wheel_effort->publish(msg);
}
}


// ── Constructor ───────────────────────────────────────────────────────────────

BalanceController::BalanceController(const rclcpp::NodeOptions & options)
: Node("balance_controller", options),
  pitch_(0.0),
  prev_pitch_(0.0),
  pitch_setpoint_(0.0),
  integral_(0.0),
  prev_error_(0.0),
  last_time_(-1.0),
  initialized_(false),
  vel_setpoint_(0.0),
  vel_integral_(0.0),
  vel_prev_error_(0.0),
  vel_last_time_(-1.0),
  vel_initialized_(false),
  vx_(0.0),
  pos_x_(0.0),
  yaw_rate_setpoint_(0.0),
  enabled_(true)
{
    // ── Inner loop parameters ─────────────────────────────────────────────────
    this->declare_parameter<double>("kp",            3.0);
    this->declare_parameter<double>("ki",            0.0);
    this->declare_parameter<double>("kd",            0.3);
    this->declare_parameter<double>("output_max",    0.20);  // Nm, effort control
    this->declare_parameter<double>("integral_max",  1.0);
    this->declare_parameter<double>("pitch_setpoint",0.0);
    this->declare_parameter<double>("pitch_limit",   1.2);
    this->declare_parameter<double>("deadband",      0.0);   // Nm, keep 0.0 at first with effort control
    this->declare_parameter<double>("dead_zone",     0.025);
    this->declare_parameter<bool>  ("publish_debug", true);

    // ── Outer loop parameters ─────────────────────────────────────────────────
    this->declare_parameter<double>("vel_kp",              0.3);
    this->declare_parameter<double>("vel_ki",              0.05);
    this->declare_parameter<double>("vel_kd",              0.01);
    this->declare_parameter<double>("vel_integral_max",    0.1);
    this->declare_parameter<double>("vel_integral_leak",   0.98);
    this->declare_parameter<double>("vx_deadzone",         0.02);
    this->declare_parameter<double>("vel_d_max",           0.05);
    this->declare_parameter<double>("pitch_setpoint_max",  0.15);  // rad

    // ── Topics ────────────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("imu_topic",     "/segway/imu/filtered");
    this->declare_parameter<std::string>("odom_topic",    "/segway/odom");
    this->declare_parameter<std::string>("cmd_vel_topic", "/segway/cmd_vel");
    this->declare_parameter<std::string>("effort_topic",  "/segway/wheel_effort_controller/commands");

    kp_           = this->get_parameter("kp").as_double();
    ki_           = this->get_parameter("ki").as_double();
    kd_           = this->get_parameter("kd").as_double();
    output_max_   = this->get_parameter("output_max").as_double();
    integral_max_ = this->get_parameter("integral_max").as_double();
    pitch_setpoint_  = this->get_parameter("pitch_setpoint").as_double();
    pitch_limit_  = this->get_parameter("pitch_limit").as_double();
    deadband_     = this->get_parameter("deadband").as_double();
    const bool publish_debug = this->get_parameter("publish_debug").as_bool();

    vel_kp_             = this->get_parameter("vel_kp").as_double();
    vel_ki_             = this->get_parameter("vel_ki").as_double();
    vel_kd_             = this->get_parameter("vel_kd").as_double();
    vel_integral_max_   = this->get_parameter("vel_integral_max").as_double();
    vx_deadzone_        = this->get_parameter("vx_deadzone").as_double();
    vel_integral_leak_  = this->get_parameter("vel_integral_leak").as_double();
    vel_d_max_          = this->get_parameter("vel_d_max").as_double();
    pitch_setpoint_max_ = this->get_parameter("pitch_setpoint_max").as_double();

    const auto imu_topic     = this->get_parameter("imu_topic").as_string();
    const auto odom_topic    = this->get_parameter("odom_topic").as_string();
    const auto cmd_vel_topic     = this->get_parameter("cmd_vel_topic").as_string();
    const auto effort_topic  = this->get_parameter("effort_topic").as_string();

    // ── QoS ───────────────────────────────────────────────────────────────────
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // ── Subscribers ───────────────────────────────────────────────────────────
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos,
        std::bind(&BalanceController::imu_callback, this, std::placeholders::_1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos,
        std::bind(&BalanceController::odom_callback, this, std::placeholders::_1));

    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic,
        10,
        std::bind(&BalanceController::cmdVelCallback, this, std::placeholders::_1));

    // ── Publishers ────────────────────────────────────────────────────────────
    g_pub_wheel_effort = this->create_publisher<std_msgs::msg::Float64MultiArray>(effort_topic, 10);

    if (publish_debug) {
        pub_pid_error_       = this->create_publisher<std_msgs::msg::Float64>(
            "/segway/debug/pid_error",       10);
        pub_pid_output_      = this->create_publisher<std_msgs::msg::Float64>(
            "/segway/debug/pid_output",      10);
        pub_vel_error_       = this->create_publisher<std_msgs::msg::Float64>(
            "/segway/debug/vel_error",       10);
        pub_pitch_setpoint_  = this->create_publisher<std_msgs::msg::Float64>(
            "/segway/debug/pitch_setpoint",  10);
    }

    // ── Outer loop timer — 50 Hz ──────────────────────────────────────────────
    vel_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&BalanceController::velocity_loop_callback, this));

    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
        "BalanceController started | "
        "Velocity source: /segway/odom ground truth | "
        "Inner: Kp=%.2f Ki=%.2f Kd=%.2f int_max=%.2f | "
        "Outer: Kp=%.3f Ki=%.4f Kd=%.4f pitch_setpoint=%.3f rad | "
        "output_max=%.2f N.m | pitch_limit=%.3f rad",
        kp_, ki_, kd_, integral_max_,
        vel_kp_, vel_ki_, vel_kd_, pitch_setpoint_,
        output_max_, pitch_limit_);
}

// ── CmdVel callback — update desired vx and desired yaw_rate ─────────────────────────────
void BalanceController::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    vel_setpoint_      = msg->linear.x;
    yaw_rate_setpoint_ = msg->angular.z;
}

// ── Odom callback — update measured vx and pos_x ─────────────────────────────

void BalanceController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const auto& q = msg->pose.pose.orientation;

    const double qw = q.w;
    const double qx = q.x;
    const double qy = q.y;
    const double qz = q.z;

    const double yaw = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz)
    );

    const double vx_world = msg->twist.twist.linear.x;
    const double vy_world = msg->twist.twist.linear.y;

    // Longitudinal ground-truth velocity in the robot forward axis.
    vx_ = std::cos(yaw) * vx_world + std::sin(yaw) * vy_world;
    prev_vx_filtered_ = vx_;
    vx_ = filter_vx(vx_);

    // angular velocity
    yaw_rate_ = msg->twist.twist.angular.z;

    pos_x_ = msg->pose.pose.position.x;
}


// ── angular velocity loop ───────────────────────────────────────────────────────

double BalanceController::compute_yaw_pid()
{
    const double yaw_error = yaw_rate_setpoint_ - yaw_rate_;

    const double p = yaw_kp_ * yaw_error;

    const double yaw_cmd = std::clamp(
        p,
        -yaw_output_max_,
        yaw_output_max_);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        100,
        "yaw_pid | yaw_sp=%.3f yaw=%.3f err=%.3f p=%.4f cmd=%.4f",
        yaw_rate_setpoint_,
        yaw_rate_,
        yaw_error,
        p,
        yaw_cmd);

    return yaw_cmd;
}


// ── Outer loop — 20 Hz ───────────────────────────────────────────────────────

void BalanceController::velocity_loop_callback()
{
    const double t_now = this->now().seconds();

    if (!vel_initialized_) {
        vel_last_time_   = t_now;
        vel_initialized_ = true;
        return;
    }

    const double dt = t_now - vel_last_time_;
    vel_last_time_ = t_now;

    if (dt <= 0.0 || dt > 1.0) return;

    // Outer PID: vx_ comes only from /segway/odom ground truth.
    // Positive vel_error means the robot is moving too much backward
    // for a zero setpoint, so the controller asks for a positive pitch setpoint.
    const double vel_error = vel_setpoint_ - vx_;

    pitch_setpoint_ = compute_vel_pid(vel_error, dt);

    // patch vx drift pb
    /* const double demo_pitch_trim = 0.003;

    if (vx_ > 0.001 && vx_ < 0.025) {
        pitch_setpoint_ -= demo_pitch_trim;
    }
    else if (vx_ < -0.001 && vx_ > -0.025) {
        pitch_setpoint_ += demo_pitch_trim;
    }
    */

    // Debug
    RCLCPP_DEBUG(this->get_logger(),
        "vel_loop | vx=%.3f setpoint=%.3f error=%.3f pitch_sp=%.4f , dt=%.4f",
        vx_, vel_setpoint_, vel_error, pitch_setpoint_, dt);

    if (pub_vel_error_ && pub_pitch_setpoint_) {
        std_msgs::msg::Float64 msg_vel_err, msg_pitch_sp;
        msg_vel_err.data  = vel_error;
        msg_pitch_sp.data = pitch_setpoint_;
        pub_vel_error_->publish(msg_vel_err);
        pub_pitch_setpoint_->publish(msg_pitch_sp);
    }
}

// ── IMU callback — inner balance loop at 100 Hz ───────────────────────────────

void BalanceController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // ── 1. dt ─────────────────────────────────────────────────────────────────
    const double t_now = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    if (!initialized_) {
        last_time_   = t_now;
        initialized_ = true;
        return;
    }

    if (!enabled_) {
        return;
    }

    const double dt = t_now - last_time_;

    if (dt < 0.0) {
        RCLCPP_WARN(this->get_logger(), "Negative dt (%.4f s) — resetting PID", dt);
        reset_pid();
        last_time_ = t_now;
        return;
    }
    if (dt > 0.5) {
        last_time_ = t_now;
        return;
    }
    last_time_ = t_now;

    // ── 2. Pitch from filtered quaternion ─────────────────────────────────────
    const double qy    = msg->orientation.y;
    const double qw    = msg->orientation.w;
    pitch_ = std::atan2(2.0 * qw * qy, 1.0 - 2.0 * qy * qy);

    // ── 3. Safety ─────────────────────────────────────────────────────────────
    if (std::abs(pitch_) > pitch_limit_) {
        if (enabled_) {
            RCLCPP_WARN(this->get_logger(),
                "Pitch limit exceeded (%.3f rad) — emergency stop", pitch_);
            enabled_ = false;
        }
        publish_wheel_effort(0.0, 0.0);
        reset_pid();
        reset_vel_pid();
        return;
    }
    // ── 4. Inner PID + angular speed PID ──────────────────────────────────────────────────────────
    const double error  = pitch_ - pitch_setpoint_;
    const double balance_effort = compute_pid(error, dt);
    const double yaw_effort = compute_yaw_pid();

    // ── 5. Deadband + publish ─────────────────────────────────────────────────
    const double left_effort = std::clamp(
        balance_effort - yaw_effort,
        -output_max_,
        output_max_);

    const double right_effort = std::clamp(
        balance_effort + yaw_effort,
        -output_max_,
        output_max_);

    publish_wheel_effort(left_effort, right_effort);

    // ── 6. outer pid  ───────────────────────────────────────────────────────── 
    double vel_error = vel_setpoint_ - vx_; 

    // ── 7. Log ────────────────────────────────────────────────────────────────

    const double t = (this->now() - start_time_).seconds();
    RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    50,
    "t=%.3f "
    "pitch=%.6f vx=%.3f pitch_sp=%.6f err=%.3f balance_effort=%.3f yaw_effort=%.3f "
    "L=%.3f R=%.3f vel_error=%.3f ",
    t,
    pitch_,
    vx_,
    pitch_setpoint_,
    error,
    balance_effort,
    yaw_effort,
    left_effort,
    right_effort,
    vel_error
    );
    if (pub_pid_error_ && pub_pid_output_) {
        std_msgs::msg::Float64 msg_error, msg_output;
        msg_error.data  = error;
        msg_output.data = balance_effort;
        pub_pid_error_->publish(msg_error);
        pub_pid_output_->publish(msg_output);
    }
}

// ── Inner PID: pitch ─────────────────────────────────────────────────────────────────

double BalanceController::compute_pid(double error, double dt)
{
    double output;

    const double p = kp_ * error;

    integral_ += error * dt;
    integral_  = std::clamp(integral_, -integral_max_, integral_max_);
    const double i = ki_ * integral_;

    // const double d = kd_ * (error - prev_error_) / dt; //  sensible aux sauts de consigne
    const double d = kd_ * (pitch_ - prev_pitch_) / dt;
    prev_error_    = error;
    prev_pitch_ = pitch_;
    output = std::clamp(p + i + d, -output_max_, output_max_); 

    
    double ff = 0.0;

    //const double t = (this->now() - start_time_).seconds();

    const bool rising_command =
        std::abs(prev_vel_setpoint_) <= drive_ff_threshold_ &&
        std::abs(vel_setpoint_) > drive_ff_threshold_;

    if (rising_command) {
        ff_active_ = true;
        ff_start_time_ = this->now();
    }

    if (ff_active_) {
        const double ff_age = (this->now() - ff_start_time_).seconds();

        if (ff_age <= drive_ff_duration_) {
            ff = -drive_ff_ * std::copysign(1.0, vel_setpoint_);
        } else {
            ff_active_ = false;
        }
    }

    prev_vel_setpoint_ = vel_setpoint_;

    output = std::clamp(output + ff, -output_max_, output_max_);
    
    RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    50,
    "inner_pid | t=%.3f pitch=%.4f sp=%.4f err=%.4f "
    "p=%.4f i=%.4f d=%.4f raw=%.4f ff=%.4f out=%.4f",
    (this->now() - start_time_).seconds(),
    pitch_,
    pitch_setpoint_,
    error,
    p,
    i,
    d,
    p + i + d,
    ff,
    output);

    return output;
}

void BalanceController::reset_pid()
{
    integral_    = 0.0;
    prev_error_  = 0.0;
    initialized_ = false;
}


double BalanceController::filter_vx(double vx)
{
    vx_window_.push_back(vx);

    while (vx_window_.size() > static_cast<size_t>(vx_moving_average_window_)) {
        vx_window_.pop_front();
    }

    double sum = 0.0;
    for (double v : vx_window_) {
        sum += v;
    }

    return sum / static_cast<double>(vx_window_.size());
}

// ── Outer PID: velocity ────────────────────────────────────────────────────────

double BalanceController::compute_vel_pid(double /*error*/, double dt)
{
    // ── 0. Sécurité dt ─────────────────────────────────────────────────────
    if (dt <= 1e-4) {
        return 0.0;
    }

    // ── 1. Rampe quantifiée sur la consigne vitesse ─────────────────────────
    const double max_delta_v = vel_accel_max_ * dt;

    const double remaining_delta_v =
        vel_setpoint_ - vel_setpoint_ramped_;

    double delta_v = std::clamp(
        remaining_delta_v,
        -max_delta_v,
        max_delta_v);

    // Marche minimale efficace : évite les pas sous la dead zone mécanique
    const double vel_step_min = 0.022;  // m/s, premier test prudent

    if (std::abs(remaining_delta_v) > vel_step_min &&
        std::abs(delta_v) < vel_step_min) {
        delta_v = std::copysign(vel_step_min, remaining_delta_v);
    }

    vel_setpoint_ramped_ += delta_v;

    // Sécurité : ne jamais dépasser la consigne cible
    if ((remaining_delta_v > 0.0 && vel_setpoint_ramped_ > vel_setpoint_) ||
        (remaining_delta_v < 0.0 && vel_setpoint_ramped_ < vel_setpoint_)) {
        vel_setpoint_ramped_ = vel_setpoint_;
    }

    const double error = vel_setpoint_ramped_ - vx_;

    const bool stop_requested =
        std::abs(vel_setpoint_) < 0.005;

    const bool stop_ramped_reached =
        std::abs(vel_setpoint_ramped_) < 0.005;

    const bool robot_almost_stopped =
        std::abs(vx_) < 0.04;

    // ── Start boost ────────────────────────────────────────────────

    const bool move_requested =
        std::abs(vel_setpoint_) > 0.005;

    const bool robot_not_moving =
        std::abs(vx_) < 0.006;

    const bool ramp_started =
        std::abs(vel_setpoint_ramped_) > 0.020;

    const double t = (this->now() - start_time_).seconds();

    if (move_requested && robot_not_moving && ramp_started && !start_boost_running_) {
        start_boost_running_ = true;
        start_boost_t0_ = t;
    }

    double start_boost = 0.0;

    if (start_boost_running_) {
        const double boost_age = t - start_boost_t0_;

        if (boost_age < start_boost_duration_ && move_requested) {
            start_boost = std::copysign(start_boost_pitch_, vel_setpoint_);
        } else {
            start_boost_running_ = false;
        }
    }


    // ── 2. Drift correction ────────────────────────────────────────────────
    if (stop_requested && stop_ramped_reached && robot_almost_stopped) {
        vx_bias_est_ =
            0.995 * vx_bias_est_ + 0.005 * vx_;
    }
    else if (!stop_requested) {
        vx_bias_est_ *= 0.98;
    }

    double pitch_offset_corr = 0.0;

    if (stop_requested &&
        stop_ramped_reached &&
        std::abs(vx_bias_est_) > 0.006) {

        pitch_offset_corr =
            std::clamp(
                -offset_kp_ * vx_bias_est_,
                -offset_max_,
                 offset_max_);
    }

    // ── 3. P ───────────────────────────────────────────────────────────────
    const double p = vel_kp_ * error;

    // ── 4. I + anti-windup ─────────────────────────────────────────────────
    vel_integral_ += error * dt;

    vel_integral_ = std::clamp(
        vel_integral_,
        -vel_integral_max_,
         vel_integral_max_);

    const bool sign_change =
        prev_vx_filtered_ * vx_ < 0.0;

    if (sign_change) {
        vel_integral_ *= 0.25;
    }
    else if (std::abs(vx_) < vx_deadzone_) {
        vel_integral_ *= 0.995;
    }

    prev_vx_filtered_ = vx_;

    const double i = vel_ki_ * vel_integral_;

    // ── 5. D sur vx ────────────────────────────────────────────────────────
    const double vx_dot_raw = (vx_ - prev_vx_) / dt;

    vx_dot_filtered_ =
        alpha_ * vx_dot_filtered_
        + (1.0 - alpha_) * vx_dot_raw;

    const double d_raw =
        -vel_kd_ * vx_dot_filtered_;

    prev_vx_ = vx_;

    const double d =
        std::clamp(d_raw, -vel_d_max_, vel_d_max_);

    vel_prev_error_ = error;

    // ── 6. Pitch setpoint ──────────────────────────────────────────────────
    double raw = p + i + d + pitch_offset_corr + start_boost;

    const double pitch_setpoint =
        std::clamp(
            raw,
            -pitch_setpoint_max_,
             pitch_setpoint_max_);

    //const double t = (this->now() - start_time_).seconds();

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        100,
        "t=%.3f "
        "outer_pid | x=%.3f x_ref=%.3f "
        "v_sp=%.3f v_sp_ramped=%.3f vx=%.4f err=%.4f "
        "p=%.4f i=%.4f pitch_offset_corr=%.4f "
        "integ=%.4f d_raw=%.4f d=%.4f "
        "raw=%.4f pitch_sp=%.4f pitch=%.4f",
        t,
        pos_x_,
        stop_x_ref_,
        vel_setpoint_,
        vel_setpoint_ramped_,
        vx_,
        error,
        p,
        i,
        pitch_offset_corr,
        vel_integral_,
        d_raw,
        d,
        raw,
        pitch_setpoint,
        pitch_);

    return pitch_setpoint;
}

void BalanceController::reset_vel_pid()
{
    vel_integral_    = 0.0;
    vel_prev_error_  = 0.0;
    vel_initialized_ = false;
}

}  // namespace segway_control

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<segway_control::BalanceController>());
    rclcpp::shutdown();
    return 0;
}
