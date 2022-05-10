#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv4/opencv2/opencv.hpp"



using namespace std::chrono_literals;
using std::placeholders::_1;

class ImagePublisher : public rclcpp::Node
{
    public:
        ImagePublisher()
        : Node("image_publisher"), count_(0)
        {
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            imgPublisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);

            imgSubscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&ImagePublisher::img_callback, this, _1));
            img2Subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera2/image_raw", 10, std::bind(&ImagePublisher::img2_callback, this, _1));

            timer_ = this->create_wall_timer(500ms, std::bind(&ImagePublisher::timer_callback,this));
        }

    private:
        void timer_callback();
        void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg);
        void img2_callback(const sensor_msgs::msg::Image::SharedPtr img_msg);
        

        rclcpp::TimerBase::SharedPtr timer_;
        //Publisher
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgPublisher_;

        //Subscriber
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSubscription_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img2Subscription_;
        size_t count_;
        
};