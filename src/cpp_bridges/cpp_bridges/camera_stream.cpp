#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

class CameraStream : public rclcpp::Node {
public:
    CameraStream() : Node("camera_stream") {
        // Subscriptions
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera1/image/image", 10,
            std::bind(&CameraStream::cb, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera1/image/depth_image", 10,
            std::bind(&CameraStream::dcb, this, std::placeholders::_1));

        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera1/image/camera_info", 10,
            std::bind(&CameraStream::icb, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar/cloud", 10,
            std::bind(&CameraStream::lidarcb, this, std::placeholders::_1));

        lidar3d_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/cloud/points", 10,
            std::bind(&CameraStream::lidar3Dcb, this, std::placeholders::_1));

        // Publishers
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera1/preprocessed_out", 10);
        repub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera1/image", 10);
        depth_repub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera1/depth_image", 10);
        info_repub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera1/camera_info", 10);
        lidar_repub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/lidar/scan", 10);
        lidar3d_repub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/scan/points", 10);
    }

private:
    // ========== Image callbacks ==========
    void cb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        auto fixed_msg = *msg;
        fixImageFrame(fixed_msg);

        // Convert to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Preprocess
        cv::Mat preprocessed = preprocess(cv_ptr->image);

        // Convert back to ROS message
        sensor_msgs::msg::Image::SharedPtr new_msg =
            cv_bridge::CvImage(msg->header, "bgr8", preprocessed).toImageMsg();

        pub_->publish(*new_msg);
    }

    void dcb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        auto new_msg = *msg;
        new_msg.header.frame_id = "camera_link";
        depth_repub_->publish(new_msg);
    }

    void icb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
        auto new_msg = *msg;
        new_msg.header.frame_id = "camera_link";
        info_repub_->publish(new_msg);
    }

    void lidarcb(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
        auto new_msg = *msg;
        new_msg.header.frame_id = "lidar_link";
        lidar_repub_->publish(new_msg);
    }

    void lidar3Dcb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
        auto new_msg = *msg;
        new_msg.header.frame_id = "lidar_link";
        lidar3d_repub_->publish(new_msg);
    }

    // ========== Helper methods ==========
    void fixImageFrame(sensor_msgs::msg::Image& msg) {
        msg.header.frame_id = "camera_link";
        repub_->publish(msg);
    }

    cv::Mat preprocess(const cv::Mat& img) {
        cv::Mat resized;
        cv::resize(img, resized, cv::Size(320, 320));
        return resized;
    }

    // ========== Members ==========
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_, depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar3d_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_, repub_, depth_repub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_repub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_repub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar3d_repub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraStream>());
    rclcpp::shutdown();
    return 0;
}
