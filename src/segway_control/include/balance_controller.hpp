#ifndef BALANCE_CONTROLLER_HPP_
#define BALANCE_CONTROLLER_HPP_

/**
 * balance_controller.hpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Boucle interne PID — équilibre du segway.
 *
 * Principe :
 *   error  = pitch_angle - pitch_setpoint   (rad)
 *   output = Kp*error + Ki*∫error dt + Kd*d(error)/dt  → cmd_vel.linear.x
 *
 * Souscrit :
 *   /segway/imu/filtered   (sensor_msgs/Imu)       — pitch angle + rate
 *   /segway/pitch_setpoint (std_msgs/Float64)       — setpoint outer loop
 *
 * Publie :
 *   /segway/cmd_vel        (geometry_msgs/Twist)    — commande moteurs
 *   /segway/debug/pid_error     (std_msgs/Float64)  — erreur courante
 *   /segway/debug/pid_output    (std_msgs/Float64)  — sortie PID
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
    void setpoint_callback(const std_msgs::msg::Float64::SharedPtr msg);

    // ── PID ───────────────────────────────────────────────────────────────────
    double compute_pid(double error, double dt);
    void   reset_pid();

    // ── Paramètres PID ────────────────────────────────────────────────────────
    double kp_;
    double ki_;
    double kd_;
    double output_max_;      // saturation sortie (m/s)
    double integral_max_;    // anti-windup
    double pitch_setpoint_;  // angle cible (rad) — mis à jour par outer loop
    double deadband_;        // seuil minimum de commande (m/s)

    // ── État PID ──────────────────────────────────────────────────────────────
    double integral_;
    double prev_error_;
    double last_time_;
    bool   initialized_;

    // ── Paramètres sécurité ───────────────────────────────────────────────────
    double pitch_limit_;     // au-delà → emergency stop (rad)
    bool   enabled_;         // le contrôleur est actif

    // ── ROS2 interfaces ───────────────────────────────────────────────────────
    // ── Position ──────────────────────────────────────────────────────────────
    double pos_x_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr  sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr    sub_imu_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr   sub_setpoint_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   pub_cmd_vel_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr      pub_pid_error_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr      pub_pid_output_;
};

}  // namespace segway_control

#endif  // BALANCE_CONTROLLER_HPP_
