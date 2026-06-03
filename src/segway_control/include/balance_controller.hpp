#ifndef BALANCE_CONTROLLER_HPP_
#define BALANCE_CONTROLLER_HPP_

/**
 * balance_controller.hpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Cascade PID — equilibrium + velocity control for the segway.
 *
 * Inner loop (100 Hz, IMU-driven):
 *   error  = pitch - pitch_setpoint   (rad)
 *   output = PID → cmd_vel.linear.x
 *
 * Outer loop (20 Hz, timer-driven):
 *   error  = vx_measured - vel_setpoint   (m/s)
 *   output = PID → pitch_setpoint (clamped to ±pitch_setpoint_max)
 *
 * Subscribes:
 *   /segway/imu/filtered   (sensor_msgs/Imu)       — filtered pitch
 *   /segway/odom           (nav_msgs/Odometry)      — measured velocity
 *   /segway/cmd_vel_user   (geometry_msgs/Twist)    — joystick setpoint
 *
 * Publishes:
 *   /segway/cmd_vel             (geometry_msgs/Twist)   — motor command
 *   /segway/debug/pid_error     (std_msgs/Float64)      — inner loop error
 *   /segway/debug/pid_output    (std_msgs/Float64)      — inner loop output
 *   /segway/debug/vel_error     (std_msgs/Float64)      — outer loop error
 *   /segway/debug/pitch_setpoint (std_msgs/Float64)     — outer loop output
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

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
    void joy_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void velocity_loop_callback();

    // ── PID helpers ───────────────────────────────────────────────────────────
    double compute_pid(double error, double dt);
    void   reset_pid();
    double compute_vel_pid(double error, double dt);
    void   reset_vel_pid();

    // ── Inner loop parameters ─────────────────────────────────────────────────
    double kp_;
    double ki_;
    double kd_;
    double output_max_;
    double integral_max_;
    double pitch_setpoint_;
    double deadband_;

    // ── Inner loop state ──────────────────────────────────────────────────────
    double integral_;
    double prev_error_;
    double last_time_;
    bool   initialized_;

    // ── Outer loop parameters ─────────────────────────────────────────────────
    double vel_kp_;
    double vel_ki_;
    double vel_kd_;
    double vel_setpoint_;           // target linear velocity (m/s), from joystick
    double pitch_setpoint_max_;     // max pitch setpoint from outer loop (rad)
    double vel_integral_max_;

    // ── Outer loop state ──────────────────────────────────────────────────────
    double vel_integral_;
    double vel_prev_error_;
    double vel_last_time_;
    bool   vel_initialized_;

    // ── Measured state ────────────────────────────────────────────────────────
    double vx_;       // measured linear velocity from odom (m/s)
    double pos_x_;    // measured position from odom (m)
    double yaw_rate_setpoint_;  // from joystick angular.z

    // ── Safety ────────────────────────────────────────────────────────────────
    double pitch_limit_;
    bool   enabled_;

    // ── ROS2 interfaces ───────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    sub_joy_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          pub_pid_error_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          pub_pid_output_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          pub_vel_error_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr          pub_pitch_setpoint_;
    rclcpp::TimerBase::SharedPtr                                  vel_timer_;
};

}  // namespace segway_control

#endif  // BALANCE_CONTROLLER_HPP_
