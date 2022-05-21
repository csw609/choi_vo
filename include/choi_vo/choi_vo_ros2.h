#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>

#include "choi_time/choi_time.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv4/opencv2/opencv.hpp"

#define FIRST_FRAME 0
#define SECOND_FRAME 1

using namespace std::chrono_literals;
using std::placeholders::_1;

// distortion
// scale
// keyframe?
// camera direction
// more feature
// image size
// image rgb->mono cvt time reduce

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

        // timer_ = this->create_wall_timer(500ms, std::bind(&ImagePublisher::timer_callback,this));
        timer_ = this->create_wall_timer(30ms, std::bind(&ImagePublisher::odom_process, this));

        // Init Information
        nFrameCount = 0;
        nMinFeatureNum = 500;
        // 640x480
        // dFx = 565.6008952774197;
        // dFy = 565.6008952774197;
        // dCx = 320.5;
        // dCy = 240.5;

        // 1920x1080
        // dFx = 1696.802685832259;
        // dFy = 1696.802685832259;
        // dCx = 960.5;
        // dCy = 540.5;

        // KITTI
        dFx = 7.18856 * 100.0;
        dFy = dFx;
        dCx = 6.071928 * 100.0;
        dCy = 1.852157 * 100.0;

        dIntrintsic[0] = dFx;
        dIntrintsic[1] = 0.0;
        dIntrintsic[2] = dCx;
        dIntrintsic[3] = 0.0;
        dIntrintsic[4] = dFy;
        dIntrintsic[5] = dCy;
        dIntrintsic[6] = 0.0;
        dIntrintsic[7] = 0.0;
        dIntrintsic[8] = 1.0;

        // No distortion case
        dDistortion[0] = 0.0;
        dDistortion[1] = 0.0;
        dDistortion[2] = 0.0;
        dDistortion[3] = 0.0;

        cvCameraMat = cv::Mat(3, 3, CV_64FC1, dIntrintsic);
        cvDistCoeffMat = cv::Mat(4, 1, CV_64FC1, dDistortion);
    }

private:
    void timer_callback();
    void img1_callback(const sensor_msgs::msg::Image::SharedPtr imgMsg_);
    void img2_callback(const sensor_msgs::msg::Image::SharedPtr imgMsg_);
    void odom_process();

    void processFirstFrame(cv::Mat &cvLeftImage);
    void processSecondFrame(cv::Mat &cvLeftImage);
    void processFrame(cv::Mat &cvLeftImage, cv::Mat &cvRightImage);

    void trackFeature(cv::Mat &cvLeftImage, std::vector<cv::Point2f> &vRefKpLeftTracked, std::vector<cv::Point2f> &vCurKpLeftTracked);
    void findRT(cv::Mat &cvRotMat, cv::Mat &cvTransMat, std::vector<cv::Point2f> &vRefKpLeftTracked, std::vector<cv::Point2f> &vCurKpLeftTracked);
    double findScale(cv::Mat &cvLeftImage, cv::Mat &cvRightImage, std::vector<cv::Point2f> &vCurKpLeftTracked);
    void visualizeFeature(cv::Mat &cvLeftImage, cv::Mat &cvRightImage);

    void readKitti(cv::Mat &cvLeftImage, cv::Mat &cvRightImage);

    rclcpp::TimerBase::SharedPtr timer_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img1Publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img2Publisher_;

    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr kittiLeftPublisher_;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr kittiRightPublisher_;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img1Subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img2Subscription_;
    size_t count_;

    // openCV
    cv_bridge::CvImage cvBridge;

    // Image Queue;
    std::queue<sensor_msgs::msg::Image::SharedPtr> left_image_buf;
    std::queue<sensor_msgs::msg::Image::SharedPtr> right_image_buf;

    // Feature Detector
    cv::Ptr<cv::FastFeatureDetector> detector_ = cv::FastFeatureDetector::create(50, true); // 25

    // Keypoints Vector
    std::vector<cv::Point2f> vRefKpLeft;
    // std::vector<cv::KeyPoint> vCurKpRight;

    // Frame
    cv::Mat cvPrevLeftImage;

    // camera information
    double dFx;
    double dFy;
    double dCx;
    double dCy;
    double dIntrintsic[9];
    double dDistortion[4];
    cv::Mat cvCameraMat;
    cv::Mat cvDistCoeffMat;

    // Frame information
    int nFrameCount;
    int nMinFeatureNum;
    cv::Mat cvCurR;
    cv::Mat cvCurT;
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
    // RCLCPP_INFO(this->get_logger(), " %s ", cv::getBuildInformation().c_str());

    seok::TimeChecker dur;

    cv::Mat cvLeftImage;
    cv::Mat cvRightImage;

    RCLCPP_INFO(this->get_logger(), "Loop Start");
    if (!left_image_buf.empty() && !right_image_buf.empty())
    {
        // double dTime;
        double dTimeLeft = left_image_buf.front()->header.stamp.sec + left_image_buf.front()->header.stamp.nanosec / 1000000000.0;
        double dTimeRight = right_image_buf.front()->header.stamp.sec + right_image_buf.front()->header.stamp.nanosec / 1000000000.0;

        double dSyncTol = 0.002; // * 1000000000;
        RCLCPP_INFO(this->get_logger(), "left time : %lf", dTimeLeft);
        RCLCPP_INFO(this->get_logger(), "right time : %lf", dTimeRight);

        if (dTimeLeft < dTimeRight - dSyncTol)
        {
            left_image_buf.pop();
            RCLCPP_INFO(this->get_logger(), "Pop Left Image");
        }
        else if (dTimeLeft > dTimeRight + dSyncTol)
        {
            right_image_buf.pop();
            RCLCPP_INFO(this->get_logger(), "Pop Right Image");
        }
        else
        {
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
            // eecdTime =
        }
    }
    if (true)
    {
        readKitti(cvLeftImage, cvRightImage);
    }

    if (!cvLeftImage.empty() && !cvRightImage.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Image Converted Well");
        RCLCPP_INFO(this->get_logger(), " frame count %d", nFrameCount);
        bool result = true;

        if (nFrameCount == FIRST_FRAME)
        {
            processFirstFrame(cvLeftImage);
        }
        else if (nFrameCount == SECOND_FRAME)
        {
            processSecondFrame(cvLeftImage);
        }
        else
        {
            processFrame(cvLeftImage, cvRightImage);
        }

        if (result)
        {
            cvPrevLeftImage = cvLeftImage;
        }

        visualizeFeature(cvLeftImage, cvRightImage);
        // RCLCPP_INFO(this->get_logger(), "size : %d", vRefKpLeft.size());
    }

    dur.interval("One Loop odomProcess");
}

void ImagePublisher::processFirstFrame(cv::Mat &cvLeftImage)
{
    std::vector<cv::KeyPoint> vKpTmp;
    detector_->detect(cvLeftImage, vKpTmp);
    vRefKpLeft.clear();
    for (auto &pts : vKpTmp)
    {
        vRefKpLeft.push_back(pts.pt);
    }

    nFrameCount++;
}

void ImagePublisher::processSecondFrame(cv::Mat &cvLeftImage)
{
    std::vector<cv::Point2f> vRefKpLeftTracked;
    std::vector<cv::Point2f> vCurKpLeftTracked;
    trackFeature(cvLeftImage, vRefKpLeftTracked, vCurKpLeftTracked);

    cv::Mat cvRotMat;
    cv::Mat cvTransMat;
    findRT(cvRotMat, cvTransMat, vRefKpLeftTracked, vCurKpLeftTracked);

    RCLCPP_INFO(this->get_logger(), "tx : %lf", cvTransMat.at<double>(0, 0));
    RCLCPP_INFO(this->get_logger(), "ty : %lf", cvTransMat.at<double>(1, 0));
    RCLCPP_INFO(this->get_logger(), "tz : %lf", cvTransMat.at<double>(2, 0));

    

    if (static_cast<int>(vCurKpLeftTracked.size()) < nMinFeatureNum)
    {
        std::vector<cv::KeyPoint> vKpTmp;
        detector_->detect(cvLeftImage, vKpTmp);
        vRefKpLeft.clear();
        for (auto &pts : vKpTmp)
        {
            vRefKpLeft.push_back(pts.pt);
        }
    }
    else
    {
        vRefKpLeft = vCurKpLeftTracked;
    }

    cvCurR = cvRotMat;
    cvCurT = cvTransMat;
    nFrameCount++;
}

void ImagePublisher::processFrame(cv::Mat &cvLeftImage, cv::Mat &cvRightImage)
{
    // RCLCPP_INFO(this->get_logger(), "process frame start");
    std::vector<cv::Point2f> vRefKpLeftTracked;
    std::vector<cv::Point2f> vCurKpLeftTracked;
    trackFeature(cvLeftImage, vRefKpLeftTracked, vCurKpLeftTracked);

    // RCLCPP_INFO(this->get_logger(), "Ref Size : %d", vCurKpLeftTmp.size());
    RCLCPP_INFO(this->get_logger(), "Cur Tracked Size : %d", vCurKpLeftTracked.size());
    RCLCPP_INFO(this->get_logger(), "Ref Tracked Size : %d", vRefKpLeftTracked.size());

    cv::Mat cvRotMat;
    cv::Mat cvTransMat;
    findRT(cvRotMat, cvTransMat, vRefKpLeftTracked, vCurKpLeftTracked);

    RCLCPP_INFO(this->get_logger(), "tx : %lf", cvTransMat.at<double>(0, 0));
    RCLCPP_INFO(this->get_logger(), "ty : %lf", cvTransMat.at<double>(1, 0));
    RCLCPP_INFO(this->get_logger(), "tz : %lf", cvTransMat.at<double>(2, 0));

    RCLCPP_INFO(this->get_logger(), " %lf %lf %lf", cvRotMat.at<double>(0, 0), cvRotMat.at<double>(0, 1), cvRotMat.at<double>(0, 2));
    RCLCPP_INFO(this->get_logger(), " %lf %lf %lf", cvRotMat.at<double>(1, 0), cvRotMat.at<double>(1, 1), cvRotMat.at<double>(1, 2));
    RCLCPP_INFO(this->get_logger(), " %lf %lf %lf", cvRotMat.at<double>(2, 0), cvRotMat.at<double>(2, 1), cvRotMat.at<double>(2, 2));

    // double scale = findScale(cvLeftImage,cvRightImage,vCurKpLeftTracked);

    seok::TimeChecker tVecTime;
    if (static_cast<int>(vCurKpLeftTracked.size()) < nMinFeatureNum)
    {
        std::vector<cv::KeyPoint> vKpTmp;
        detector_->detect(cvLeftImage, vKpTmp);
        vRefKpLeft.clear();
        for (auto &pts : vKpTmp)
        {
            vRefKpLeft.push_back(pts.pt);
        }
    }
    else
    {
        vRefKpLeft = vCurKpLeftTracked;
    }

    tVecTime.interval("vector copy");
    nFrameCount++;
}

double ImagePublisher::findScale(cv::Mat &cvLeftImage, cv::Mat &cvRightImage, std::vector<cv::Point2f> &vCurKpLeftTracked)
{
    std::vector<cv::Point2f> vCurKpRight;
    std::vector<uchar> vCurStatus;
    cv::calcOpticalFlowPyrLK(cvLeftImage, cvRightImage, vCurKpLeftTracked, vCurKpRight, vCurStatus, cv::noArray(), cv::Size(21, 21), 3);

    double scale = 0.0;

    int nVecSize = static_cast<int>(vCurStatus.size());
    for (int i = 0; i < nVecSize; i++)
    {
        if (vCurStatus[i])
        { // if tracked;
            double dLeftX = vCurKpLeftTracked[i].x;
            double dLeftY = vCurKpLeftTracked[i].y;

            double dRightX = vCurKpRight[i].x;
            double dRightY = vCurKpRight[i].y;

            if (dLeftX - dRightX < 1)
                continue; // skip outlier with too small disparity
            // if()
        }
    }

    return scale;
}

void ImagePublisher::findRT(cv::Mat &cvRotMat, cv::Mat &cvTransMat, std::vector<cv::Point2f> &vRefKpLeftTracked, std::vector<cv::Point2f> &vCurKpLeftTracked)
{

    // later consider distortion
    seok::TimeChecker tFindTime;
    cv::Mat cvEsMat;
    cvEsMat = cv::findEssentialMat(vCurKpLeftTracked, vRefKpLeftTracked, cvCameraMat); //, cv::RANSAC, 0.999, 1.0);
    tFindTime.interval("Find Essen");
    seok::TimeChecker tReTime;
    cv::recoverPose(cvEsMat, vCurKpLeftTracked, vRefKpLeftTracked, cvCameraMat, cvRotMat, cvTransMat);
    tReTime.interval("recover");
}

void ImagePublisher::trackFeature(cv::Mat &cvLeftImage, std::vector<cv::Point2f> &vRefKpLeftTracked, std::vector<cv::Point2f> &vCurKpLeftTracked)
{
    std::vector<cv::Point2f> vCurKpLeftTmp;
    std::vector<uchar> vCurStatus;
    // std::vector<float>        vErr;

    seok::TimeChecker tTrackTime;
    cv::calcOpticalFlowPyrLK(cvPrevLeftImage, cvLeftImage, vRefKpLeft, vCurKpLeftTmp, vCurStatus, cv::noArray(), cv::Size(21, 21), 3); // 1,
    // cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01));

    int nStatSize = static_cast<int>(vCurStatus.size());
    for (int i = 0; i < nStatSize; i++)
    {
        if (vCurStatus[i]) // if tracked
        {
            vCurKpLeftTracked.push_back(vCurKpLeftTmp[i]);
            vRefKpLeftTracked.push_back(vRefKpLeft[i]);
        }
    }
    tTrackTime.interval("Feature Tracking");
}

void ImagePublisher::visualizeFeature(cv::Mat &cvLeftImage, cv::Mat &cvRightImage)
{
    cv::Mat cvVisualLeft = cvLeftImage;
    for (auto &pts : vRefKpLeft)
    {
        cv::circle(cvVisualLeft, cv::Point(static_cast<int>(pts.x), static_cast<int>(pts.y)), 5, cv::Scalar(255, 0, 255), 2, 4, 0);
    }

    std_msgs::msg::Header header;

    cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, cvVisualLeft);
    sensor_msgs::msg::Image imageVisual;
    cvBridge.toImageMsg(imageVisual);

    cvBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, cvRightImage);
    sensor_msgs::msg::Image imageVisual2;
    cvBridge.toImageMsg(imageVisual2);

    img1Publisher_->publish(imageVisual);
    // img2Publisher_->publish(imageVisual2);
}

void ImagePublisher::img1_callback(const sensor_msgs::msg::Image::SharedPtr imgMsg_)
{
    // RCLCPP_INFO(this->get_logger(), "img1 subscribed");
    left_image_buf.push(imgMsg_);
}

void ImagePublisher::img2_callback(const sensor_msgs::msg::Image::SharedPtr imgMsg_)
{
    RCLCPP_INFO(this->get_logger(), "img2 subscribed");
    right_image_buf.push(imgMsg_);
}

void ImagePublisher::readKitti(cv::Mat &cvLeftImage, cv::Mat &cvRightImage)
{
    std::string img_l_path;
    std::string img_r_path;
    // read image
    if (nFrameCount < 10)
    {
        img_l_path = "/home/csw/dataset/KITTI/sequence/00/image_0/00000" + std::to_string(nFrameCount) + ".png";
        img_r_path = "/home/csw/dataset/KITTI/sequence/00/image_1/00000" + std::to_string(nFrameCount) + ".png";
    }
    else if (nFrameCount < 100)
    {
        img_l_path = "/home/csw/dataset/KITTI/sequence/00/image_0/0000" + std::to_string(nFrameCount) + ".png";
        img_r_path = "/home/csw/dataset/KITTI/sequence/00/image_1/0000" + std::to_string(nFrameCount) + ".png";
    }
    else if (nFrameCount < 1000)
    {
        img_l_path = "/home/csw/dataset/KITTI/sequence/00/image_0/000" + std::to_string(nFrameCount) + ".png";
        img_r_path = "/home/csw/dataset/KITTI/sequence/00/image_1/000" + std::to_string(nFrameCount) + ".png";
    }
    else if (nFrameCount < 10000)
    {
        img_l_path = "/home/csw/dataset/KITTI/sequence/00/image_0/00" + std::to_string(nFrameCount) + ".png";
        img_r_path = "/home/csw/dataset/KITTI/sequence/00/image_0/00" + std::to_string(nFrameCount) + ".png";
    }

    cvLeftImage = cv::imread(img_l_path, cv::IMREAD_GRAYSCALE);
    cvRightImage = cv::imread(img_r_path, cv::IMREAD_GRAYSCALE);
}