/**
 * balance_controller.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Implémentation boucle interne PID équilibre segway.
 * Voir balance_controller.hpp pour la documentation.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "balance_controller.hpp"

#include <cmath>
#include <algorithm>

namespace segway_control
{

// ── Constructeur ─────────────────────────────────────────────────────────────

BalanceController::BalanceController(const rclcpp::NodeOptions & options)
: Node("balance_controller", options),
  integral_(0.0),
  prev_error_(0.0),
  last_time_(-1.0),
  initialized_(false),
  enabled_(true)
{
    // ── Paramètres PID ────────────────────────────────────────────────────────
    this->declare_parameter<double>("kp",            10.0);
    this->declare_parameter<double>("ki",             0.5);
    this->declare_parameter<double>("kd",             0.1);
    this->declare_parameter<double>("output_max",     0.5);
    this->declare_parameter<double>("integral_max",   1.0);
    this->declare_parameter<double>("pitch_setpoint", 0.0);
    this->declare_parameter<double>("pitch_limit",    0.5);
    this->declare_parameter<double>("deadband",       0.05);  // m/s
    this->declare_parameter<bool>  ("publish_debug",  true);

    // Topics
    this->declare_parameter<std::string>("imu_topic",      "/segway/imu/filtered");
    this->declare_parameter<std::string>("setpoint_topic", "/segway/pitch_setpoint");
    this->declare_parameter<std::string>("cmd_vel_topic",  "/segway/cmd_vel");

    kp_             = this->get_parameter("kp").as_double();
    ki_             = this->get_parameter("ki").as_double();
    kd_             = this->get_parameter("kd").as_double();
    output_max_     = this->get_parameter("output_max").as_double();
    integral_max_   = this->get_parameter("integral_max").as_double();
    pitch_setpoint_ = this->get_parameter("pitch_setpoint").as_double();
    pitch_limit_    = this->get_parameter("pitch_limit").as_double();
    deadband_       = this->get_parameter("deadband").as_double();
    const bool publish_debug = this->get_parameter("publish_debug").as_bool();

    const auto imu_topic      = this->get_parameter("imu_topic").as_string();
    const auto setpoint_topic = this->get_parameter("setpoint_topic").as_string();
    const auto cmd_vel_topic  = this->get_parameter("cmd_vel_topic").as_string();

    // ── QoS ───────────────────────────────────────────────────────────────────
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // ── Subscribers ───────────────────────────────────────────────────────────
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos,
        std::bind(&BalanceController::imu_callback, this, std::placeholders::_1)
    );

    sub_setpoint_ = this->create_subscription<std_msgs::msg::Float64>(
        setpoint_topic, 10,
        std::bind(&BalanceController::setpoint_callback, this, std::placeholders::_1)
    );

    pos_x_ = 0.0;
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/segway/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            pos_x_ = msg->pose.pose.position.x;
        });

    // ── Publishers ────────────────────────────────────────────────────────────
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
        cmd_vel_topic, 10);

    if (publish_debug) {
        pub_pid_error_  = this->create_publisher<std_msgs::msg::Float64>(
            "/segway/debug/pid_error",  10);
        pub_pid_output_ = this->create_publisher<std_msgs::msg::Float64>(
            "/segway/debug/pid_output", 10);
    }

    RCLCPP_INFO(this->get_logger(),
        "BalanceController démarré | Kp=%.2f Ki=%.2f Kd=%.2f | "
        "output_max=%.2f m/s | deadband=%.3f m/s | pitch_limit=%.3f rad",
        kp_, ki_, kd_, output_max_, deadband_, pitch_limit_);
}

// ── Callback IMU ─────────────────────────────────────────────────────────────

void BalanceController::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // ── 1. Calcul de dt ───────────────────────────────────────────────────────
    const double t_now =
        msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    if (!initialized_) {
        last_time_   = t_now;
        initialized_ = true;
        return;
    }

    const double dt = t_now - last_time_;

    if (dt < 0.0) {
        RCLCPP_WARN(this->get_logger(),
            "dt négatif (%.4f s) — réinitialisation PID", dt);
        reset_pid();
        last_time_ = t_now;
        return;
    }
    if (dt > 0.5) {
        last_time_ = t_now;
        return;
    }
    last_time_ = t_now;

    // ── 2. Extraction pitch depuis quaternion filtré ───────────────────────────
    //    Le filtre IMU publie orientation = euler_to_quaternion(0, pitch, 0)
    //    On récupère pitch via : pitch = 2 * asin(q.y)  [roll=0, yaw=0]
    const double qy    = msg->orientation.y;
    const double qw    = msg->orientation.w;
    const double pitch = std::atan2(2.0 * qw * qy, 1.0 - 2.0 * qy * qy);

    // ── 3. Sécurité — emergency stop si trop incliné ──────────────────────────
    if (std::abs(pitch) > pitch_limit_) {
        if (enabled_) {
            RCLCPP_WARN(this->get_logger(),
                "Angle limite dépassé (%.3f rad) — emergency stop", pitch);
            enabled_ = false;
        }
        // Publier cmd_vel = 0
        pub_cmd_vel_->publish(geometry_msgs::msg::Twist());
        reset_pid();
        return;
    }
    // Réactiver si revenu dans les limites (intervention manuelle)
    if (!enabled_) {
        RCLCPP_INFO(this->get_logger(), "Angle OK — contrôleur réactivé");
        enabled_ = true;
    }

    // ── 4. PID ────────────────────────────────────────────────────────────────
    const double error  = pitch - pitch_setpoint_;
    const double output = compute_pid(error, dt);

    // ── 5. Publication cmd_vel ────────────────────────────────────────────────
    //    Segway : si incliné en avant (pitch > 0) → avancer pour rattraper
    //    Zone morte : si |output| < deadband → pousser au minimum deadband
    //    linear.x  = sortie PID
    //    angular.z = 0 (boucle externe gèrera la rotation)
    double cmd_x = output;
    if (std::abs(output) > 1e-6 && std::abs(output) < deadband_) {
        cmd_x = std::copysign(deadband_, output);
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = cmd_x;
    cmd.angular.z = 0.0;
    pub_cmd_vel_->publish(cmd);

    // ── 6. Debug ──────────────────────────────────────────────────────────────
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "pitch=%.3f error=%.3f output=%.3f cmd=%.3f pos_x=%.3f sat=%s db=%s",
        pitch, error, output, cmd_x, pos_x_,
        std::abs(output) >= output_max_ ? "YES" : "no",
        std::abs(output) > 1e-6 && std::abs(output) < deadband_ ? "YES" : "no");
        
    if (pub_pid_error_ && pub_pid_output_) {
        std_msgs::msg::Float64 msg_error, msg_output;
        msg_error.data  = error;
        msg_output.data = output;
        pub_pid_error_->publish(msg_error);
        pub_pid_output_->publish(msg_output);
    }
}

// ── Callback setpoint (outer loop) ───────────────────────────────────────────

void BalanceController::setpoint_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    pitch_setpoint_ = msg->data;
}

// ── PID ──────────────────────────────────────────────────────────────────────

double BalanceController::compute_pid(double error, double dt)
{
    // Proportionnel
    const double p = kp_ * error;

    // Intégral avec anti-windup (clamping)
    integral_ += error * dt;
    integral_  = std::clamp(integral_, -integral_max_, integral_max_);
    const double i = ki_ * integral_;

    // Dérivée (sur l'erreur, pas sur le setpoint → évite derivative kick)
    const double d = kd_ * (error - prev_error_) / dt;
    prev_error_    = error;

    // Sortie saturée
    const double output = std::clamp(p + i + d, -output_max_, output_max_);
    return output;
}

void BalanceController::reset_pid()
{
    integral_    = 0.0;
    prev_error_  = 0.0;
    initialized_ = false;
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
