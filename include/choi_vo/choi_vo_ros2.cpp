#include "choi_vo/choi_vo_ros2.h"

// void ImagePublisher::timer_callback() {
//     auto message = std_msgs::msg::String();
//     message.data = "Hello World!" + std::to_string(count_++);
//     RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//     publisher_->publish(message);
// }

void ImagePublisher::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello World!" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img(640, 480, CV_8UC3, cv::Scalar(50, 100, 200));

    // auto img_msg = sensor_msgs::msg::Image();

    sensor_msgs::msg::Image::SharedPtr img_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
    // img_publisher_
    imgPublisher_->publish(*img_msg.get());
}

void ImagePublisher::img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    RCLCPP_INFO(this->get_logger(), "img2 subscribed");
}

void ImagePublisher::img2_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    RCLCPP_INFO(this->get_logger(), "img2 subscribed");
}