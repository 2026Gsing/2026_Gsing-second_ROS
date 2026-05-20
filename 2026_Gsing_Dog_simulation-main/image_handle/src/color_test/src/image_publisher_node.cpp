#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>  // 改为 .hpp
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <string>
#include <memory>

using namespace std::chrono_literals;

class ImagePublisherNode : public rclcpp::Node
{
public:
    ImagePublisherNode(const std::string& image_path = "") 
        : Node("image_publisher_node"), 
          use_camera_(image_path.empty()),
          image_path_(image_path),
          camera_index_(0)
    {
        // 创建图像发布者
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/my_camera/image_raw", 10);  // 修改话题名

        if (use_camera_)
        {
            // 使用摄像头
            RCLCPP_INFO(this->get_logger(), "Initializing camera (index: %d)", camera_index_);
            cap_.open(camera_index_);
            
            if (!cap_.isOpened())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera with index %d", camera_index_);
                return;
            }

            // 设置摄像头参数（可选）
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
            cap_.set(cv::CAP_PROP_FPS, 30);

            RCLCPP_INFO(this->get_logger(), "Camera opened successfully");
            
            // 创建定时器，定期从摄像头读取并发布图像
            timer_ = this->create_wall_timer(
                33ms,  // 约30fps
                std::bind(&ImagePublisherNode::camera_timer_callback, this));
        }
        else
        {
            // 使用图像文件
            RCLCPP_INFO(this->get_logger(), "Loading image from: %s", image_path_.c_str());
            image_ = cv::imread(image_path_);
            
            if (image_.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load image from: %s", image_path_.c_str());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Image loaded successfully. Size: %dx%d", 
                       image_.cols, image_.rows);

            // 创建定时器，定期发布图像文件
            timer_ = this->create_wall_timer(
                100ms,  // 10fps for static image
                std::bind(&ImagePublisherNode::image_file_timer_callback, this));
        }
    }

    ~ImagePublisherNode()
    {
        if (cap_.isOpened())
        {
            cap_.release();
        }
    }

private:
    void camera_timer_callback()
    {
        if (!cap_.isOpened())
        {
            return;
        }

        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Failed to capture frame from camera");
            return;
        }

        publish_image(frame);
    }

    void image_file_timer_callback()
    {
        if (image_.empty())
        {
            return;
        }

        publish_image(image_);
    }

    void publish_image(const cv::Mat& cv_image)
    {
        try
        {
            // 将OpenCV图像转换为ROS2图像消息
            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = "camera_frame";

            sensor_msgs::msg::Image::SharedPtr img_msg = 
                cv_bridge::CvImage(header, "bgr8", cv_image).toImageMsg();

            // 发布图像
            image_publisher_->publish(*img_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool use_camera_;
    std::string image_path_;
    int camera_index_;
    cv::VideoCapture cap_;
    cv::Mat image_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::string image_path = "";
    
    // 解析命令行参数
    if (argc > 1)
    {
        image_path = std::string(argv[1]);
        RCLCPP_INFO(rclcpp::get_logger("image_publisher"), 
                   "Image path provided: %s", image_path.c_str());
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("image_publisher"), 
                   "No image path provided, using camera");
    }

    auto node = std::make_shared<ImagePublisherNode>(image_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}