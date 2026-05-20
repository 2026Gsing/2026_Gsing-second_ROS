#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cv_bridge/cv_bridge.hpp>  // 改为 .hpp
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <memory>
#include <string>

class ColorDetectionNode : public rclcpp::Node
{
public:
    ColorDetectionNode() : Node("color_detection_node")
    {
        // 订阅图像话题 - 改为 /my_camera/image_raw 以匹配桥接配置
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/my_camera/image_raw", 10,
            std::bind(&ColorDetectionNode::image_callback, this, std::placeholders::_1));

        // 创建发布者
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // 创建高分区发布者（用于通知 path_planner）
        high_score_zone_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/high_score_zone", 10);

        // 定义RGB掩膜颜色（BGR格式，因为OpenCV使用BGR）
        // 食品: RGB(117, 189, 66) -> BGR(66, 189, 117)
        food_color_ = cv::Scalar(66, 189, 117);
        // 工具: RGB(128, 128, 128) -> BGR(128, 128, 128)
        tool_color_ = cv::Scalar(128, 128, 128);
        // 药箱: RGB(200, 29, 49) -> BGR(49, 29, 200)
        medicine_color_ = cv::Scalar(49, 29, 200);
        // 仪器: RGB(46, 84, 161) -> BGR(161, 84, 46)
        instrument_color_ = cv::Scalar(161, 84, 46);

        // 颜色阈值（允许的偏差范围）
        color_threshold_ = 30;

        RCLCPP_INFO(this->get_logger(), "Color Detection Node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /my_camera/image_raw");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 将ROS图像消息转换为OpenCV格式
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;

            // 转换为灰度图用于角点检测
            cv::Mat gray;
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

            // 检测角点
            std::vector<cv::Point2f> corners = detect_corners(gray);

            auto twist_msg = geometry_msgs::msg::Twist();

            if (corners.size() == 4)
            {
                // 透视变换为正方形
                cv::Mat warped = perspective_transform(image, corners);

                // 在变换后的图像上进行颜色识别
                std::string detected_object = detect_color(warped);
                
                // 根据识别结果设置高分区
                set_high_score_zone(detected_object);

                // 运动控制部分
                // 计算角点中心位置
                cv::Point2f center(0, 0);
                for (const auto& pt : corners) {
                    center += pt;
                }
                center *= 0.25; // 除以4

                // 计算画面中心
                float img_center_x = image.cols / 2.0;
                // 计算偏移量
                float error_x = center.x - img_center_x;
                // P控制器参数
                float k_p = 0.005;
                // 设置角速度
                twist_msg.angular.z = -k_p * error_x;
                // 设置线速度
                if (std::abs(error_x) < 50) {
                    twist_msg.linear.x = 0.2; // 前进速度 0.2 m/s
                } else {
                    twist_msg.linear.x = 0.05; // 缓慢前进或原地调整
                }
                
                // 绘制
                cv::circle(image, center, 5, cv::Scalar(0, 0, 255), -1); 
                cv::line(image, cv::Point(img_center_x, 0), cv::Point(img_center_x, image.rows), cv::Scalar(255, 255, 0), 2); 

                RCLCPP_INFO(this->get_logger(), "Detected object: %s", detected_object.c_str());

                cv::imshow("Original", image);
                cv::imshow("Warped", warped);
                cv::waitKey(1);
            }
            else
            {
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.0; 
                
                RCLCPP_DEBUG(this->get_logger(), "No target detected.");
                cv::imshow("Camera View", image);
                cv::waitKey(1);
            }
            cmd_vel_publisher_->publish(twist_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void set_high_score_zone(const std::string& object_type)
    {
        auto msg = std_msgs::msg::Int32();
        
        // 根据识别的物体类型设置高分区ID
        if (object_type == "食品") {
            msg.data = 0;  // 食品区对应ID 0
        } else if (object_type == "工具") {
            msg.data = 1;  // 工具区对应ID 1
        } else if (object_type == "仪器") {
            msg.data = 2;  // 仪器区对应ID 2
        } else if (object_type == "药箱") {
            msg.data = 3;  // 药品区对应ID 3
        } else {
            return;  // 未知类型不发布
        }
        
        high_score_zone_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published high score zone: %d (%s)", msg.data, object_type.c_str());
    }

    std::vector<cv::Point2f> detect_corners(const cv::Mat& gray)
    {
        std::vector<cv::Point2f> corners;

        // 使用Shi-Tomasi角点检测
        cv::goodFeaturesToTrack(gray, corners, 4, 0.01, 100);

        // 如果检测到的角点不是4个，尝试使用轮廓检测
        if (corners.size() != 4)
        {
            // 使用Canny边缘检测
            cv::Mat edges;
            cv::Canny(gray, edges, 50, 150);

            // 查找轮廓
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (!contours.empty())
            {
                // 找到最大的轮廓
                auto largest_contour = std::max_element(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                        return cv::contourArea(a) < cv::contourArea(b);
                    });

                // 近似轮廓为多边形
                std::vector<cv::Point> approx;
                cv::approxPolyDP(*largest_contour, approx, 0.02 * cv::arcLength(*largest_contour, true), true);

                if (approx.size() == 4)
                {
                    corners.clear();
                    for (const auto& pt : approx)
                    {
                        corners.push_back(cv::Point2f(static_cast<float>(pt.x), static_cast<float>(pt.y)));
                    }
                }
            }
        }

        // 对角点进行排序：左上、右上、左下、右下
        if (corners.size() == 4)
        {
            corners = sort_corners(corners);
        }

        return corners;
    }

    std::vector<cv::Point2f> sort_corners(std::vector<cv::Point2f> corners)
    {
        // 计算中心点
        cv::Point2f center(0, 0);
        for (const auto& corner : corners)
        {
            center += corner;
        }
        center *= (1.0 / corners.size());

        // 根据与中心点的位置关系排序
        std::vector<cv::Point2f> sorted(4);
        for (const auto& corner : corners)
        {
            if (corner.x < center.x && corner.y < center.y)
                sorted[0] = corner; // 左上
            else if (corner.x > center.x && corner.y < center.y)
                sorted[1] = corner; // 右上
            else if (corner.x < center.x && corner.y > center.y)
                sorted[2] = corner; // 左下
            else
                sorted[3] = corner; // 右下
        }

        return sorted;
    }

    cv::Mat perspective_transform(const cv::Mat& image, const std::vector<cv::Point2f>& corners)
    {
        // 定义目标正方形的尺寸（使用原始图像的平均宽度和高度）
        float width = std::max(
            cv::norm(corners[0] - corners[1]),
            cv::norm(corners[2] - corners[3])
        );
        float height = std::max(
            cv::norm(corners[0] - corners[2]),
            cv::norm(corners[1] - corners[3])
        );

        int size = static_cast<int>(std::max(width, height));

        // 定义目标正方形的四个角点
        std::vector<cv::Point2f> dst_corners = {
            cv::Point2f(0, 0),
            cv::Point2f(static_cast<float>(size), 0),
            cv::Point2f(0, static_cast<float>(size)),
            cv::Point2f(static_cast<float>(size), static_cast<float>(size))
        };

        // 计算透视变换矩阵
        cv::Mat M = cv::getPerspectiveTransform(corners, dst_corners);

        // 应用透视变换
        cv::Mat warped;
        cv::warpPerspective(image, warped, M, cv::Size(size, size));

        return warped;
    }

    std::string detect_color(const cv::Mat& image)
    {
        // 将图像分成多个区域进行检测
        int rows = image.rows;
        int cols = image.cols;

        // 定义检测区域（可以调整区域大小）
        int region_size = std::min(rows, cols) / 4;
        
        // 在图像中心区域进行检测
        int center_x = cols / 2;
        int center_y = rows / 2;
        int half_size = region_size / 2;

        cv::Rect center_region(
            center_x - half_size,
            center_y - half_size,
            region_size,
            region_size
        );

        // 检查每个颜色掩膜
        double food_score = calculate_color_match(image(center_region), food_color_);
        double tool_score = calculate_color_match(image(center_region), tool_color_);
        double medicine_score = calculate_color_match(image(center_region), medicine_color_);
        double instrument_score = calculate_color_match(image(center_region), instrument_color_);

        // 找到匹配度最高的颜色
        std::vector<std::pair<double, std::string>> scores = {
            {food_score, "食品"},
            {tool_score, "工具"},
            {medicine_score, "药箱"},
            {instrument_score, "仪器"}
        };

        auto max_score = std::max_element(scores.begin(), scores.end(),
            [](const std::pair<double, std::string>& a, const std::pair<double, std::string>& b) {
                return a.first < b.first;
            });

        // 如果匹配度超过阈值，返回对应的类别
        if (max_score->first > 0.5)  // 50%的匹配度阈值
        {
            return max_score->second;
        }

        return "未知";
    }

    double calculate_color_match(const cv::Mat& region, const cv::Scalar& target_color)
    {
        if (region.empty())
            return 0.0;

        // 计算区域的平均颜色
        cv::Scalar mean_color = cv::mean(region);

        // 计算与目标颜色的欧氏距离（归一化到0-1）
        double diff_b = std::abs(mean_color[0] - target_color[0]);
        double diff_g = std::abs(mean_color[1] - target_color[1]);
        double diff_r = std::abs(mean_color[2] - target_color[2]);

        double distance = std::sqrt(diff_b * diff_b + diff_g * diff_g + diff_r * diff_r);
        
        // 归一化到0-1，距离越小匹配度越高
        double max_distance = std::sqrt(3 * 255 * 255);
        double match_score = 1.0 - (distance / max_distance);

        // 如果距离在阈值内，返回更高的匹配度
        if (distance < color_threshold_)
        {
            match_score = 1.0 - (distance / color_threshold_) * 0.3;  // 在阈值内时匹配度更高
        }

        return match_score;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr high_score_zone_publisher_;

    cv::Scalar food_color_;
    cv::Scalar tool_color_;
    cv::Scalar medicine_color_;
    cv::Scalar instrument_color_;
    int color_threshold_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ColorDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}