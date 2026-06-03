/**
 * imu_filter_node.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Implémentation du filtre complémentaire IMU.
 * Voir imu_filter_node.hpp pour la documentation.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "imu_filter_node.hpp"

#include <cmath>

namespace segway_control
{

// ── Constructeur ─────────────────────────────────────────────────────────────

ImuFilterNode::ImuFilterNode(const rclcpp::NodeOptions & options)
: Node("imu_filter_node", options),
  pitch_angle_(0.0),
  last_time_(-1.0),
  initialized_(false)
{
    // ── Paramètres ────────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("imu_topic",    "/segway/imu/data");
    this->declare_parameter<std::string>("output_topic", "/segway/imu/filtered");
    this->declare_parameter<double>("alpha",         0.98);
    this->declare_parameter<double>("pitch_offset",  0.0);
    this->declare_parameter<bool>  ("publish_debug", true);

    const auto imu_topic    = this->get_parameter("imu_topic").as_string();
    const auto output_topic = this->get_parameter("output_topic").as_string();
    alpha_        = this->get_parameter("alpha").as_double();
    pitch_offset_ = this->get_parameter("pitch_offset").as_double();
    publish_debug_= this->get_parameter("publish_debug").as_bool();

    // ── QoS ───────────────────────────────────────────────────────────────────
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // ── Subscriber ────────────────────────────────────────────────────────────
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, qos,
        std::bind(&ImuFilterNode::imu_callback, this, std::placeholders::_1)
    );

    // ── Publishers ────────────────────────────────────────────────────────────
    pub_filtered_ = this->create_publisher<sensor_msgs::msg::Imu>(
        output_topic, 10);

    if (publish_debug_) {
        pub_pitch_raw_      = this->create_publisher<std_msgs::msg::Float64>(
            "/segway/debug/pitch_raw",      10);
        pub_pitch_accel_    = this->create_publisher<std_msgs::msg::Float64>(
            "/segway/debug/pitch_accel",    10);
        pub_pitch_filtered_ = this->create_publisher<std_msgs::msg::Float64>(
            "/segway/debug/pitch_filtered", 10);
    }

    RCLCPP_INFO(this->get_logger(),
        "ImuFilterNode démarré | alpha=%.3f | pitch_offset=%.4f rad",
        alpha_, pitch_offset_);
}

// ── Callback ─────────────────────────────────────────────────────────────────

void ImuFilterNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
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

    // dt négatif → horloge Gazebo resetée ou message désynchronisé
    // dt > 0.5  → pause simulation trop longue → réinitialiser sans planter
    if (dt < 0.0) {
        RCLCPP_WARN(this->get_logger(),
            "dt négatif (%.4f s) — horloge sim resetée, réinitialisation", dt);
        last_time_   = t_now;
        pitch_angle_ = 0.0;
        return;
    }
    if (dt > 0.5) {
        RCLCPP_WARN(this->get_logger(),
            "dt trop grand (%.4f s) — pause sim détectée, message ignoré", dt);
        last_time_ = t_now;
        return;
    }

    last_time_ = t_now;

    // ── 2. Gyroscope : pitch rate ω (rad/s) ──────────────────────────────────
    //    Axe Y = axe de tangage dans le frame imu_link
    const double omega = msg->angular_velocity.y;

    // ── 3. Accéléromètre : angle géométrique (rad) ───────────────────────────
    //    Mesure directe de l'inclinaison via la direction du vecteur gravité
    //    Pas d'intégration — atan2(ax, az) seul
    const double ax = msg->linear_acceleration.x;
    const double az = msg->linear_acceleration.z;

    const double pitch_accel = (std::abs(az) > 1e-9)
        ? std::atan2(-ax, az)
        : 0.0;

    // ── 4. Filtre complémentaire ──────────────────────────────────────────────
    //
    //   θ[n] = α × (θ[n-1] + ω × dt)   ← gyro intégré
    //        + (1-α) × atan2(ax, az)    ← accéléro direct
    //
    const double pitch_gyro = pitch_angle_ + omega * dt;
    pitch_angle_ = alpha_       * pitch_gyro
                 + (1.0 - alpha_) * pitch_accel
                 - pitch_offset_;

    // ── 5. Publication ────────────────────────────────────────────────────────
    sensor_msgs::msg::Imu out;
    out.header          = msg->header;
    out.header.frame_id = "imu_link";

    // Orientation : quaternion depuis pitch filtré (roll=0, yaw=0)
    out.orientation = euler_to_quaternion(0.0, pitch_angle_, 0.0);
    out.orientation_covariance[0] = 1e6;   // roll  : inconnu
    out.orientation_covariance[4] = 0.01;  // pitch : estimé
    out.orientation_covariance[8] = 1e6;   // yaw   : inconnu

    // Gyro et accéléro retransmis tels quels
    out.angular_velocity            = msg->angular_velocity;
    out.angular_velocity_covariance = msg->angular_velocity_covariance;
    out.linear_acceleration            = msg->linear_acceleration;
    out.linear_acceleration_covariance = msg->linear_acceleration_covariance;

    pub_filtered_->publish(out);

    // ── 6. Debug ──────────────────────────────────────────────────────────────
    if (publish_debug_) {
        std_msgs::msg::Float64 msg_raw, msg_accel, msg_filtered;
        msg_raw.data      = pitch_gyro;
        msg_accel.data    = pitch_accel;
        msg_filtered.data = pitch_angle_;
        pub_pitch_raw_->publish(msg_raw);
        pub_pitch_accel_->publish(msg_accel);
        pub_pitch_filtered_->publish(msg_filtered);
    }
}

// ── Helper ────────────────────────────────────────────────────────────────────

geometry_msgs::msg::Quaternion ImuFilterNode::euler_to_quaternion(
    double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll  * 0.5);
    const double sr = std::sin(roll  * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw   * 0.5);
    const double sy = std::sin(yaw   * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

}  // namespace segway_control

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<segway_control::ImuFilterNode>());
    rclcpp::shutdown();
    return 0;
}
