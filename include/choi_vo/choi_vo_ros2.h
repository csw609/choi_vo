#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv4/opencv2/opencv.hpp"

#define FIRST_FRAME 0
#define SECOND_FRAME 1

using namespace std::chrono_literals;
using std::placeholders::_1;

class ImagePublisher : public rclcpp::Node
{
    public:
        ImagePublisher()
        : Node("image_publisher"), count_(0)
        {
            RCLCPP_INFO(this->get_logger(), "choi VO Init!");
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
            img1Publisher_ = this->create_publisher<sensor_msgs::msg::Image>("left_image", 10);
            img2Publisher_ = this->create_publisher<sensor_msgs::msg::Image>("right_image", 10);

            img1Subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&ImagePublisher::img1_callback, this, _1));
            img2Subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera2/image_raw", 10, std::bind(&ImagePublisher::img2_callback, this, _1));

            //timer_ = this->create_wall_timer(500ms, std::bind(&ImagePublisher::timer_callback,this));
            timer_ = this->create_wall_timer(100ms, std::bind(&ImagePublisher::odom_process,this));


            //Init Information
            nFrameCount = 0;
        }

    private:
        void timer_callback();
        void img1_callback(const sensor_msgs::msg::Image::SharedPtr imgMsg_);
        void img2_callback(const sensor_msgs::msg::Image::SharedPtr imgMsg_);
        void odom_process();

        void processFirstFrame(cv::Mat &cvImage);
        void processSecondFrame(cv::Mat &cvImage);
        void processFrame(cv::Mat &cvImage);

        void visualizeFeature(cv::Mat &cvImage1);
        

        rclcpp::TimerBase::SharedPtr timer_;

        //Publisher
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img1Publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img2Publisher_;

        //Subscriber
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img1Subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img2Subscription_;
        size_t count_;

        // openCV
        cv_bridge::CvImage cvBridge;

        // Image Queue;
        std::queue<sensor_msgs::msg::Image::SharedPtr> left_image_buf;
        std::queue<sensor_msgs::msg::Image::SharedPtr> right_image_buf;

        // Feature Detector
        cv::Ptr<cv::FastFeatureDetector> detector_ = cv::FastFeatureDetector::create(10, true); // 25

        // Keypoints Vector
        std::vector<cv::Point2f> vRefKpLeft;
        //std::vector<cv::KeyPoint> vCurKpRight;

        //Frame
        cv::Mat cvPrevLeftImage;

        // Frame Count
        int nFrameCount;

        
};

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
    img1Publisher_->publish(*img_msg.get());
}

void ImagePublisher::odom_process()
{
    RCLCPP_INFO(this->get_logger(), "Loop Start");
    if (!left_image_buf.empty() && !right_image_buf.empty())
    {
        //double dTime;
        double dTimeLeft = left_image_buf.front()->header.stamp.sec +  left_image_buf.front()->header.stamp.nanosec / 1000000000.0;
        double dTimeRight = right_image_buf.front()->header.stamp.sec + right_image_buf.front()->header.stamp.nanosec / 1000000000.0;

        cv::Mat cvLeftImage;
        cv::Mat cvRightImage;
        double dSyncTol = 0.002;// * 1000000000;
        RCLCPP_INFO(this->get_logger(), "left time : %lf", dTimeLeft);
        RCLCPP_INFO(this->get_logger(), "right time : %lf", dTimeRight);

        if(dTimeLeft < dTimeRight - dSyncTol){
            left_image_buf.pop();
            RCLCPP_INFO(this->get_logger(), "Pop Left Image");
        }
        else if(dTimeLeft > dTimeRight + dSyncTol){
            right_image_buf.pop();
            RCLCPP_INFO(this->get_logger(), "Pop Right Image");
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Sync!");
            cv_bridge::CvImageConstPtr bridgeLeft_;
            cv_bridge::CvImageConstPtr bridgeRight_;
            
            bridgeLeft_ = cv_bridge::toCvCopy(left_image_buf.front(), sensor_msgs::image_encodings::BGR8);
            bridgeRight_ = cv_bridge::toCvCopy(right_image_buf.front(), sensor_msgs::image_encodings::BGR8);

            cvLeftImage = bridgeLeft_->image.clone();

            cv::cvtColor(cvLeftImage, cvLeftImage, cv::COLOR_BGR2GRAY);
            
            left_image_buf.pop();

            cvRightImage = bridgeRight_->image.clone();
            right_image_buf.pop();
            //eecdTime = 
        }

        if(!cvLeftImage.empty() && !cvRightImage.empty()){
            RCLCPP_INFO(this->get_logger(), "Image Converted Well");
            RCLCPP_INFO(this->get_logger(), " frame count %d", nFrameCount);
            bool result = true;

            if(nFrameCount == FIRST_FRAME){
                processFirstFrame(cvLeftImage);
            }
            else if(nFrameCount == SECOND_FRAME){
                processSecondFrame(cvLeftImage);
            }
            else{
                processFrame(cvLeftImage);
            }

            if(result){
                cvPrevLeftImage = cvLeftImage;
            }
            
            
            visualizeFeature(cvLeftImage);
            RCLCPP_INFO(this->get_logger(), "size : %d", vRefKpLeft.size());
            

        }
    }
}

void ImagePublisher::processFirstFrame(cv::Mat &cvImage){
    std::vector<cv::KeyPoint> vKpTmp; 
    detector_-> detect(cvImage, vKpTmp);
    vRefKpLeft.clear();
    for(auto &pts : vKpTmp){
        vRefKpLeft.push_back(pts.pt);
    }

    nFrameCount++;
}

void ImagePublisher::processSecondFrame(cv::Mat &cvImage){
    std::vector<cv::Point2f>  vCurKpLeftTmp;
    std::vector<cv::Point2f>  vCurKpLeftTracked;
    std::vector<cv::Point2f>  vRefKpLeftTracked;
    std::vector<uchar>        vCurStatus;
    std::vector<float>        vErr;
    
    cv::calcOpticalFlowPyrLK(cvPrevLeftImage, cvImage, vRefKpLeft, vCurKpLeftTmp, vCurStatus, cv::noArray());//, cv::Size(21, 21), 3); //1,
    //cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01));

    int nStatSize = static_cast<int>(vCurStatus.size());
    for(int i = 0; i <  nStatSize; i++){
        if(vCurStatus[i]) // if tracked
        {
            vCurKpLeftTracked.push_back(vCurKpLeftTmp[i]);
            vRefKpLeftTracked.push_back(vRefKpLeft[i]);
        }
    }

    //vRefKpLeft = vCurKpLeft;

    nFrameCount++;
}

void ImagePublisher::processFrame(cv::Mat &cvImage){
    //RCLCPP_INFO(this->get_logger(), "process frame start");
    std::vector<cv::Point2f>  vCurKpLeftTmp;
    std::vector<cv::Point2f>  vCurKpLeftTracked;
    std::vector<cv::Point2f>  vRefKpLeftTracked;
    std::vector<uchar>        vCurStatus;
    std::vector<float>        vErr;
    
    cv::calcOpticalFlowPyrLK(cvPrevLeftImage, cvImage, vRefKpLeft, vCurKpLeftTmp, vCurStatus, cv::noArray());//, cv::Size(21, 21), 3); //1,
    //cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01));

    int nStatSize = static_cast<int>(vCurStatus.size());
    for(int i = 0; i <  nStatSize; i++){
        if(vCurStatus[i]) // if tracked
        {
            vCurKpLeftTracked.push_back(vCurKpLeftTmp[i]);
            vRefKpLeftTracked.push_back(vRefKpLeft[i]);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Ref Size : %d", vCurKpLeftTmp.size());
    RCLCPP_INFO(this->get_logger(), "Cur Tracked Size : %d", vCurKpLeftTracked.size());
    RCLCPP_INFO(this->get_logger(), "Ref Tracked Size : %d", vRefKpLeftTracked.size());

    //vRefKpLeft = vCurKpLeft;

    nFrameCount++;

}

void ImagePublisher::visualizeFeature(cv::Mat &cvImage1){
    cv::Mat cvVisualLeft = cvImage1;
    for(auto &pts : vRefKpLeft){
        cv::circle(cvVisualLeft, cv::Point(static_cast<int>(pts.x), static_cast<int>(pts.y)), 5, cv::Scalar(255, 0, 255), 2, 4, 0);
    }

    std_msgs::msg::Header header;
    cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, cvVisualLeft);
    
    sensor_msgs::msg::Image imageVisual;
    cvBridge.toImageMsg(imageVisual);

    img1Publisher_->publish(imageVisual);
}

void ImagePublisher::img1_callback(const sensor_msgs::msg::Image::SharedPtr imgMsg_)
{
    //RCLCPP_INFO(this->get_logger(), "img1 subscribed");
    left_image_buf.push(imgMsg_);
}

void ImagePublisher::img2_callback(const sensor_msgs::msg::Image::SharedPtr imgMsg_)
{
    RCLCPP_INFO(this->get_logger(), "img2 subscribed");
    right_image_buf.push(imgMsg_);
}