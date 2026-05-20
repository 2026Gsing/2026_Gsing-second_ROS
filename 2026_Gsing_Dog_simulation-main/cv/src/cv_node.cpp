#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"        // ROS 2 标准图像消息
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/highgui.hpp"              // 用于 cv::imshow
#include "opencv2/imgproc.hpp"              // (可选) 用于未来的图像处理

// 1. 定义你的节点类，它继承自 rclcpp::Node
class CvNode : public rclcpp::Node
{
public:
  CvNode() : Node("cv_node")
  {
    // 2. 创建一个图像订阅者 (Subscriber)
    // 话题名称必须和你在 camera.urdf.xacro 插件中定义的 <image_topic_name> 一致
    // (我们之前用的是 /my_camera/image_raw)
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/my_camera/image_raw",  // 话题名称
      10,                      // QoS (队列深度)
      std::bind(&CvNode::image_callback, this, std::placeholders::_1)); // 回调函数

    RCLCPP_INFO(this->get_logger(), "CV 节点已启动，正在订阅 /my_camera/image_raw...");
  }

private:
  // 4. 这是订阅者收到消息时执行的回调函数
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      // 5. [核心] 将 ROS 图像消息 (msg) 转换为 OpenCV 图像 (Mat)
      // "bgr8" 是 OpenCV 的标准颜色编码
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat frame = cv_ptr->image;

      // 如果帧是空的，就不要处理
      if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "收到空的图像帧");
        return;
      }

      // 6. [验证] 在窗口中显示图像
      // 这是最简单的验证方法，证明你收到了数据
      cv::imshow("Gazebo Camera Feed", frame);
      
      // (注意: 你之后要做的所有颜色识别代码，都写在这里)

      // 7. cv::waitKey(1) 是 cv::imshow 必须的
      cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换异常: %s", e.what());
    }
  }

  // 3. 声明订阅者成员变量
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

// 8. ROS 2 节点的主函数
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  // 创建并“旋转” (spin) 节点，使其保持活动状态以接收消息
  rclcpp::spin(std::make_shared<CvNode>());
  rclcpp::shutdown();
  cv::destroyAllWindows(); // 关闭 OpenCV 窗口
  return 0;
}