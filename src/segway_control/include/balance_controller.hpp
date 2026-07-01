#ifndef BALANCE_CONTROLLER_HPP_
#define BALANCE_CONTROLLER_HPP_

/**
 * balance_controller.hpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Cascade PID — equilibrium + velocity control for the segway.
 *
 * Inner loop (100 Hz, IMU-driven):
 *   error  = pitch - pitch_setpoint   (rad)
 *   output = PID → wheel effort
 *
 * Outer loop (50 Hz, timer-driven):
 *   error  = vel_setpoint - vx_measured   (m/s)
 *   output = PID → pitch_setpoint (clamped to ±pitch_setpoint_max)
 *
 * Subscribes:
 *   /segway/imu/filtered  (sensor_msgs/Imu)     — filtered pitch
 *   /segway/odom          (nav_msgs/Odometry)   — measured velocity
 *   /segway/cmd_vel       (geometry_msgs/Twist) — joystick setpoint
 *
 * Publishes:
 *   /segway/wheel_effort_controller/commands (std_msgs/Float64MultiArray)
 *   /segway/debug/pid_error                  (std_msgs/Float64)
 *   /segway/debug/pid_output                 (std_msgs/Float64)
 *   /segway/debug/vel_error                  (std_msgs/Float64)
 *   /segway/debug/pitch_setpoint             (std_msgs/Float64)
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <deque>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

namespace segway_control
{

class BalanceController : public rclcpp::Node
{
public:
    explicit BalanceController(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // ── Callbacks ─────────────────────────────────────────────────────────────
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void velocity_loop_callback();

    // ── PID helpers ───────────────────────────────────────────────────────────
    double compute_pid(double error, double dt);
    double compute_vel_pid(double error, double dt);
    double compute_yaw_pid();
    double filter_vx(double vx);

    void reset_pid();
    void reset_vel_pid();

    // ── Inner loop parameters ─────────────────────────────────────────────────
    double kp_;
    double ki_;
    double kd_;
    double output_max_;
    double integral_max_;
    double pitch_setpoint_;
    double deadband_;

    // ── Inner loop state ──────────────────────────────────────────────────────
    double pitch_;
    double prev_pitch_;
    double integral_;
    double prev_error_;
    double last_time_;
    bool initialized_;

    // ── Outer loop parameters ─────────────────────────────────────────────────
    double vel_kp_;
    double vel_ki_;
    double vel_kd_;
    double pitch_setpoint_max_;
    double vel_integral_max_;
    double vel_d_max_;
    double vx_deadzone_;
    double vel_integral_leak_;

    // ── Outer loop state ──────────────────────────────────────────────────────
    double vel_setpoint_;
    double vel_setpoint_ramped_ = 0.0;
    double vel_accel_max_ = 0.04;
    double vel_integral_;
    double vel_prev_error_;
    double vel_last_time_;
    bool vel_initialized_;

    // ── Measured state ────────────────────────────────────────────────────────
    double vx_;
    double pos_x_;
    double yaw_rate_ = 0.0;
    double yaw_rate_setpoint_ = 0.0;

    // ── Safety ────────────────────────────────────────────────────────────────
    double pitch_limit_;
    bool enabled_;

    // ── Logging ───────────────────────────────────────────────────────────────
    rclcpp::Time start_time_;

    // ── Velocity filtering ────────────────────────────────────────────────────
    std::deque<double> vx_window_;
    int vx_moving_average_window_ = 3;
    double prev_vx_filtered_ = 0.0;

    double alpha_ = 0.99;
    double prev_vx_ = 0.0;
    double vx_dot_filtered_ = 0.0;

    // ── Stop / drift correction ───────────────────────────────────────────────
    double stop_x_ref_ = 0.0;
    double stop_hold_kp_ = 0.10;
    double stop_hold_vmax_ = 0.010;
    double stop_hold_counter_ = 0.0;

    double vx_bias_est_ = 0.0;
    double offset_kp_ = 0.45;
    double offset_max_ = 0.008;

    // ── Feedforward / start boost ─────────────────────────────────────────────
    double prev_vel_setpoint_ = 0.0;
    rclcpp::Time ff_start_time_;
    bool ff_active_ = false;

    double drive_ff_ = 0.010;
    double drive_ff_duration_ = 0.25;
    double drive_ff_threshold_ = 0.005;

    double start_boost_pitch_ = 0.0015;
    double start_boost_duration_ = 0.25;
    double start_boost_t0_ = -1.0;
    bool start_boost_running_ = false;

    // ── Yaw loop ──────────────────────────────────────────────────────────────
    double yaw_kp_ = 0.05;
    double yaw_output_max_ = 0.05;

    // ── ROS 2 interfaces ──────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pid_error_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pid_output_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_vel_error_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pitch_setpoint_;

    rclcpp::TimerBase::SharedPtr vel_timer_;
};

}  // namespace segway_control

#endif  // BALANCE_CONTROLLER_HPP_
