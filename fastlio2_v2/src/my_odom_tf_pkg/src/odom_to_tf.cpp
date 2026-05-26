/**
 * odom_to_tf.cpp — 里程计 → TF 变换广播节点
 *
 * 功能：
 *   订阅 /odom 话题（Odometry 消息），将里程计位姿作为
 *   odom→base_link 的 TF 变换广播出去。
 *
 * 用途：
 *   当 FAST-LIO2 或其他里程计源发布 /Odometry 但不广播 TF 时，
 *   此节点充当桥接，使 Nav2 和其他依赖 TF 的组件能正常工作。
 *
 * TF 关系：
 *   odom (父) → base_link (子)
 */

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdomToTF : public rclcpp::Node {
public:
    OdomToTF() : Node("odom_to_tf") {
        // 订阅 /odom 话题（或 FAST-LIO2 的 /Odometry），接收里程计数据
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&OdomToTF::odom_callback, this, std::placeholders::_1));

        // 初始化 TF 广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 将 Odometry 消息转换为 TF 变换并广播

        geometry_msgs::msg::TransformStamped transform;

        // 使用消息时间戳，保证 TF 与传感器数据时间对齐
        transform.header.stamp = msg->header.stamp;
        // 坐标系：odom（父）→ base_link（子）
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        // 从 Odometry 消息中提取位置和姿态
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;

        // 广播 TF 变换
        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToTF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
