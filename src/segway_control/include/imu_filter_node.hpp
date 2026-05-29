#ifndef IMU_FILTER_NODE_HPP_
#define IMU_FILTER_NODE_HPP_

/**
 * imu_filter_node.hpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Filtre complémentaire IMU pour segway auto-équilibré.
 *
 * θ[n] = α × (θ[n-1] + ω_gyro × dt) + (1-α) × atan2(ax, az)
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace segway_control
{

class ImuFilterNode : public rclcpp::Node
{
public:
    explicit ImuFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // ── Callback ──────────────────────────────────────────────────────────────
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // ── Helper ────────────────────────────────────────────────────────────────
    static geometry_msgs::msg::Quaternion euler_to_quaternion(
        double roll, double pitch, double yaw);

    // ── Paramètres ────────────────────────────────────────────────────────────
    double alpha_;           // coefficient filtre complémentaire
    double pitch_offset_;    // correction angle repos (rad)
    bool   publish_debug_;   // active topics debug

    // ── État interne ──────────────────────────────────────────────────────────
    double pitch_angle_;     // angle filtré courant (rad)
    double last_time_;       // timestamp dernier message (s)
    bool   initialized_;     // faux jusqu'au premier message

    // ── ROS2 interfaces ───────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    pub_filtered_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr   pub_pitch_raw_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr   pub_pitch_accel_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr   pub_pitch_filtered_;
};

}  // namespace segway_control

#endif  // IMU_FILTER_NODE_HPP_
