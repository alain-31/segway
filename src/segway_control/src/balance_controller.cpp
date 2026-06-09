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
    this->declare_parameter<double>("pitch_limit",   1.2);
    this->declare_parameter<double>("deadband",      0.0);   // Nm, keep 0.0 at first with effort control
    this->declare_parameter<double>("dead_zone",     0.025);
     this->declare_parameter<bool>  ("publish_debug", true);

    // ── Outer loop parameters ─────────────────────────────────────────────────
    this->declare_parameter<double>("vel_kp",              0.3);
    this->declare_parameter<double>("vel_ki",              0.05);
    this->declare_parameter<double>("vel_kd",              0.01);
    this->declare_parameter<double>("vel_integral_max",    0.1);
    this->declare_parameter<double>("pitch_setpoint_max",  0.15);  // rad

    // ── Topics ────────────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("imu_topic",     "/segway/imu/filtered");
    this->declare_parameter<std::string>("odom_topic",    "/segway/odom");
    this->declare_parameter<std::string>("joy_topic",     "/segway/cmd_vel_user");
    this->declare_parameter<std::string>("effort_topic",  "/segway/wheel_effort_controller/commands");

    kp_           = this->get_parameter("kp").as_double();
    ki_           = this->get_parameter("ki").as_double();
    kd_           = this->get_parameter("kd").as_double();
    output_max_   = this->get_parameter("output_max").as_double();
    integral_max_ = this->get_parameter("integral_max").as_double();
    pitch_limit_  = this->get_parameter("pitch_limit").as_double();
    deadband_     = this->get_parameter("deadband").as_double();
    const bool publish_debug = this->get_parameter("publish_debug").as_bool();

    vel_kp_             = this->get_parameter("vel_kp").as_double();
    vel_ki_             = this->get_parameter("vel_ki").as_double();
    vel_kd_             = this->get_parameter("vel_kd").as_double();
    vel_integral_max_   = this->get_parameter("vel_integral_max").as_double();
    pitch_setpoint_max_ = this->get_parameter("pitch_setpoint_max").as_double();

    const auto imu_topic     = this->get_parameter("imu_topic").as_string();
    const auto odom_topic    = this->get_parameter("odom_topic").as_string();
    const auto joy_topic     = this->get_parameter("joy_topic").as_string();
    const auto effort_topic  = this->get_parameter("effort_topic").as_string();

    // ── QoS ───────────────────────────────────────────────────────────────────
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // ── Subscribers ───────────────────────────────────────────────────────────
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos,
        std::bind(&BalanceController::imu_callback, this, std::placeholders::_1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&BalanceController::odom_callback, this, std::placeholders::_1));

    sub_joy_ = this->create_subscription<geometry_msgs::msg::Twist>(
        joy_topic, 10,
        std::bind(&BalanceController::joy_callback, this, std::placeholders::_1));

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

    RCLCPP_INFO(this->get_logger(),
        "BalanceController started | "
        "Inner: Kp=%.2f Ki=%.2f Kd=%.2f int_max=%.2f| "
        "Outer: Kp=%.2f Ki=%.3f Kd=%.3f pitch_max=%.3f rad | "
        "output_max=%.2f N.m | pitch_limit=%.3f rad",
        kp_, ki_, kd_, integral_max_,
        vel_kp_, vel_ki_, vel_kd_, pitch_setpoint_max_,
        output_max_,pitch_limit_);
}

// ── Odom callback — update measured vx and pos_x ─────────────────────────────

void BalanceController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    vx_    = msg->twist.twist.linear.x;
    pos_x_ = msg->pose.pose.position.x;
}

// ── Joystick callback — update velocity and yaw setpoints ────────────────────

void BalanceController::joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    vel_setpoint_      = msg->linear.x;
    yaw_rate_setpoint_ = msg->angular.z;
    reset_pid();  // reset inner integrator on new velocity command
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

    // Outer PID: error = measured_vx - desired_vx
    // Positive error (going too fast forward) → negative pitch setpoint (lean back)
    const double vel_error = vel_setpoint_ -  vx_ ;
    pitch_setpoint_ = compute_vel_pid(vel_error, dt);

    // Debug
    RCLCPP_DEBUG(this->get_logger(),
        "vel_loop | vx=%.3f setpoint=%.3f error=%.3f pitch_sp=%.4f",
        vx_, vel_setpoint_, vel_error, pitch_setpoint_);

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
    const double pitch = std::atan2(2.0 * qw * qy, 1.0 - 2.0 * qy * qy);

    // ── 3. Safety ─────────────────────────────────────────────────────────────
    if (std::abs(pitch) > pitch_limit_) {
        if (enabled_) {
            RCLCPP_WARN(this->get_logger(),
                "Pitch limit exceeded (%.3f rad) — emergency stop", pitch);
            enabled_ = false;
        }
        publish_wheel_effort(0.0, 0.0);
        reset_pid();
        reset_vel_pid();
        return;
    }
    if (!enabled_) {
        RCLCPP_INFO(this->get_logger(), "Pitch OK — controller re-enabled");
        enabled_ = true;
    }

    // ── 4. Inner PID ──────────────────────────────────────────────────────────
    const double error  = pitch - pitch_setpoint_;
    const double output = compute_pid(error, dt);

    // ── 5. Deadband + publish ─────────────────────────────────────────────────
    const double balance_effort = output;
    const double yaw_effort     = yaw_rate_setpoint_;

    const double left_effort  = balance_effort - yaw_effort;
    const double right_effort = balance_effort + yaw_effort;

    publish_wheel_effort(left_effort, right_effort);

    // ── 6. Log ────────────────────────────────────────────────────────────────
RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    50,
    "pitch=%.3f err=%.3f pid=%.3f bal=%.3f yaw=%.3f "
    "L=%.3f R=%.3f sat=%s",
    pitch,
    error,
    output,
    balance_effort,
    yaw_effort,
    left_effort,
    right_effort,
    std::abs(output) >= output_max_ ? "YES" : "no");

    if (pub_pid_error_ && pub_pid_output_) {
        std_msgs::msg::Float64 msg_error, msg_output;
        msg_error.data  = error;
        msg_output.data = output;
        pub_pid_error_->publish(msg_error);
        pub_pid_output_->publish(msg_output);
    }
}

// ── Inner PID ─────────────────────────────────────────────────────────────────

double BalanceController::compute_pid(double error, double dt)
{
    const double p = kp_ * error;

    integral_ += error * dt;
    integral_  = std::clamp(integral_, -integral_max_, integral_max_);
    const double i = ki_ * integral_;

    const double d = kd_ * (error - prev_error_) / dt;
    prev_error_    = error;

    return std::clamp(p + i + d, -output_max_, output_max_);
}

void BalanceController::reset_pid()
{
    integral_    = 0.0;
    prev_error_  = 0.0;
    initialized_ = false;
}

// ── Outer velocity PID ────────────────────────────────────────────────────────

double BalanceController::compute_vel_pid(double error, double dt)
{
    const double p = vel_kp_ * error;

    vel_integral_ += error * dt;
    vel_integral_  = std::clamp(vel_integral_, -vel_integral_max_, vel_integral_max_);
    const double i = vel_ki_ * vel_integral_;

    const double d = vel_kd_ * (error - vel_prev_error_) / dt;
    vel_prev_error_ = error;

    // Output is pitch_setpoint — clamped to safety limit
    return std::clamp(p + i + d, -pitch_setpoint_max_, pitch_setpoint_max_);
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
