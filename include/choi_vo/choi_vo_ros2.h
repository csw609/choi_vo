#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include "choi_time/choi_time.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv4/opencv2/opencv.hpp"

#define FIRST_FRAME 0
#define SECOND_FRAME FIRST_FRAME + 1

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
        pathPublisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);
        gtPathPublisher_ = this->create_publisher<nav_msgs::msg::Path>("path_gt", 10);

        img1Subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&ImagePublisher::img1_callback, this, _1));
        img2Subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera2/image_raw", 10, std::bind(&ImagePublisher::img2_callback, this, _1));

        // timer_ = this->create_wall_timer(500ms, std::bind(&ImagePublisher::timer_callback,this));
        timer_ = this->create_wall_timer(100ms, std::bind(&ImagePublisher::odom_process, this));

        // Init Information
        nFrameCount = FIRST_FRAME;
        nMinFeatureNum = 100;

        // Detector
        bUseFAST = false;
        bUseGFTT = true;
        if (bUseFAST)
        {
            detector_ = cv::FastFeatureDetector::create(20, true); // 25
        }
        else if (bUseGFTT)
        {
            nMaxFeatureNumGFTT = 300;
            nMinDist = 30;
            dQualityLev = 0.01;

            // KITTI parameter
            // nMaxFeatureNumGFTT = 300;
            // nMinDist = 30;
            // dQualityLev = 0.01;
        }

        // recoverPose
        dRansacThresh = 1.0;

        // Simulation
        if (true)
        {
            // 640x480
            dFx = 565.6008952774197;
            dFy = 565.6008952774197;
            dCx = 320.5;
            dCy = 240.5;

            // 1920x1080
            // dFx = 1696.802685832259;
            // dFy = 1696.802685832259;
            // dCx = 960.5;
            // dCy = 540.5;
            // dBaseline = 0.1; (m)
        }
        else
        {

            // KITTI
            // dFx = 7.18856 * 100.0;
            // dFy = dFx;
            // dCx = 6.071928 * 100.0;
            // dCy = 1.852157 * 100.0;

            // dIntrintsic[0] = dFx;
            // dIntrintsic[1] = 0.0;
            // dIntrintsic[2] = dCx;
            // dIntrintsic[3] = 0.0;
            // dIntrintsic[4] = dFy;
            // dIntrintsic[5] = dCy;
            // dIntrintsic[6] = 0.0;
            // dIntrintsic[7] = 0.0;
            // dIntrintsic[8] = 1.0;
        }

        // No distortion case
        dDistortion[0] = 0.0;
        dDistortion[1] = 0.0;
        dDistortion[2] = 0.0;
        dDistortion[3] = 0.0;

        cvCameraMat = cv::Mat(3, 3, CV_64FC1, dIntrintsic);
        cvCameraMatIv = cvCameraMat.inv();
        cvDistCoeffMat = cv::Mat(4, 1, CV_64FC1, dDistortion);

        dBaseline = 0.54;

        fsGroundTruth.open("/home/csw/dataset/KITTI/sequence/00/00_gt.txt");
        for (int i = 0; i < nFrameCount; i++)
        {
            std::string strGT;
            std::getline(fsGroundTruth, strGT);
        }

        cvCurT = cv::Mat::zeros(3, 1, CV_64FC1);
        cvCurR = cv::Mat::eye(3, 3, CV_64FC1);

        msgPath.header.frame_id = "camera_optical";
        msgGtPath.header.frame_id = "camera_optical";
    }

private:
    void timer_callback();
    void img1_callback(const sensor_msgs::msg::Image::SharedPtr imgMsg_);
    void img2_callback(const sensor_msgs::msg::Image::SharedPtr imgMsg_);
    void odom_process();

    void processFirstFrame(cv::Mat &cvLeftImage);
    void processSecondFrame(cv::Mat &cvLeftImage);
    bool processFrame(cv::Mat &cvLeftImage, cv::Mat &cvRightImage);

    void detectFeature(cv::Mat &cvImage, std::vector<cv::Point2f> &vKp);
    void trackFeature(cv::Mat &cvLeftImage, std::vector<cv::Point2f> &vRefKpLeftTracked, std::vector<cv::Point2f> &vCurKpLeftTracked);

    void findRT(cv::Mat &cvRotMat, cv::Mat &cvTransMat, std::vector<cv::Point2f> &vRefKpLeftTracked, std::vector<cv::Point2f> &vCurKpLeftTracked);
    double findScale(cv::Mat &cvLeftImage, cv::Mat &cvRightImage, std::vector<cv::Point2f> &vRefKpLeftTracked, std::vector<cv::Point2f> &vCurKpLeftTracked, cv::Mat &cvRotMat, cv::Mat &cvTransMat);

    void visualizeFeature(cv::Mat &cvLeftImage, cv::Mat &cvRightImage);
    void visualizePath();

    void readKitti(cv::Mat &cvLeftImage, cv::Mat &cvRightImage);
    void readKittiGT();

    rclcpp::TimerBase::SharedPtr timer_;

    // Publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img1Publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img2Publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gtPathPublisher_;

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
    cv::Ptr<cv::FastFeatureDetector> detector_;
    bool bUseFAST;
    bool bUseGFTT; // Good Feature To Track

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

    double dBaseline;

    cv::Mat cvCameraMat;
    cv::Mat cvCameraMatIv;
    cv::Mat cvDistCoeffMat;

    // Frame information
    int nFrameCount;
    int nMinFeatureNum;
    int nMaxFeatureNumGFTT;
    int nMinDist;
    double dQualityLev;
    double dRansacThresh;
    cv::Mat cvCurR;
    cv::Mat cvCurT;
    nav_msgs::msg::Path msgPath;
    nav_msgs::msg::Path msgGtPath;

    // source
    bool bUseCamera;
    bool bUseKITTI;
    std::ifstream fsGroundTruth;
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

    // RCLCPP_INFO(this->get_logger(), "Loop Start");
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
    if (false   )
    {
        readKitti(cvLeftImage, cvRightImage);
        readKittiGT();
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
        // else if (nFrameCount == SECOND_FRAME)
        // {
        //     processSecondFrame(cvLeftImage);
        // }
        else
        {
            processFrame(cvLeftImage, cvRightImage);
            visualizePath();
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

    detectFeature(cvLeftImage, vRefKpLeft);
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
        detectFeature(cvLeftImage, vRefKpLeft);
    }
    else
    {
        vRefKpLeft = vCurKpLeftTracked;
    }

    cvCurR = cvRotMat;
    cvCurT = cvTransMat;
    nFrameCount++;
}

bool ImagePublisher::processFrame(cv::Mat &cvLeftImage, cv::Mat &cvRightImage)
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

    RCLCPP_INFO(this->get_logger(), "tx : %lf, ty : %lf, tz : %lf", cvTransMat.at<double>(0, 0), cvTransMat.at<double>(1, 0), cvTransMat.at<double>(2, 0));

    // RCLCPP_INFO(this->get_logger(), " %lf %lf %lf", cvRotMat.at<double>(0, 0), cvRotMat.at<double>(0, 1), cvRotMat.at<double>(0, 2));
    // RCLCPP_INFO(this->get_logger(), " %lf %lf %lf", cvRotMat.at<double>(1, 0), cvRotMat.at<double>(1, 1), cvRotMat.at<double>(1, 2));
    // RCLCPP_INFO(this->get_logger(), " %lf %lf %lf", cvRotMat.at<double>(2, 0), cvRotMat.at<double>(2, 1), cvRotMat.at<double>(2, 2));

    double dScale = findScale(cvLeftImage, cvRightImage, vRefKpLeftTracked, vCurKpLeftTracked, cvRotMat, cvTransMat);

    if (!std::isnan(dScale) && dScale > 0.1 && dScale < 5.0)
    {

        cvCurT = cvCurT + dScale * cvCurR * cvTransMat;
        cvCurR = cvCurR * cvRotMat;
    }

    seok::TimeChecker tVecTime;
    if (static_cast<int>(vCurKpLeftTracked.size()) < nMinFeatureNum)
    {
        detectFeature(cvLeftImage, vRefKpLeft);
    }
    else
    {
        vRefKpLeft = vCurKpLeftTracked;
    }

    tVecTime.interval("vector copy");
    nFrameCount++;

    return true;
}

double ImagePublisher::findScale(cv::Mat &cvLeftImage, cv::Mat &cvRightImage, std::vector<cv::Point2f> &vRefKpLeftTracked, std::vector<cv::Point2f> &vCurKpLeftTracked, cv::Mat &cvRotMat, cv::Mat &cvTransMat)
{
    double dScale = 1.0;
    // estimate monocular depth

    std::vector<cv::Point2f> vRefKpLeftNorm;
    std::vector<cv::Point2f> vCurKpLeftNorm;
    cv::Mat cvMonoDep4D;
    // std::vector<cv::Vec4f> vMonoDep4D;

    cv::Mat cvProjId = cv::Mat::eye(3, 4, CV_64FC1); // Ref is ref
    cv::Mat cvProjP2C;                               // projection matrix Identity, prev to cur
    cv::Mat cvTransP2C = -cvRotMat.t() * cvTransMat;
    cv::hconcat(cvRotMat.t(), cvTransP2C, cvProjP2C);

    // std::cout << cvProjP2C << std::endl;

    // RCLCPP_INFO(this->get_logger(), " Camera Inv");
    // RCLCPP_INFO(this->get_logger(), " %lf %lf %lf", cvCameraMatIv.at<double>(0, 0), cvCameraMatIv.at<double>(0, 1), cvCameraMatIv.at<double>(0, 2));
    // RCLCPP_INFO(this->get_logger(), " %lf %lf %lf", cvCameraMatIv.at<double>(1, 0), cvCameraMatIv.at<double>(1, 1), cvCameraMatIv.at<double>(1, 2));
    // RCLCPP_INFO(this->get_logger(), " %lf %lf %lf", cvCameraMatIv.at<double>(2, 0), cvCameraMatIv.at<double>(2, 1), cvCameraMatIv.at<double>(2, 2));

    int nVecSize = static_cast<int>(vRefKpLeftTracked.size());
    for (int i = 0; i < nVecSize; i++)
    {
        cv::Mat cvRef(3, 1, CV_64FC1);
        cvRef.at<double>(0, 0) = vRefKpLeftTracked[i].x;
        cvRef.at<double>(1, 0) = vRefKpLeftTracked[i].y;
        cvRef.at<double>(2, 0) = 1.0;

        cv::Mat cvCur(3, 1, CV_64FC1);
        cvCur.at<double>(0, 0) = vCurKpLeftTracked[i].x;
        cvCur.at<double>(1, 0) = vCurKpLeftTracked[i].y;
        cvCur.at<double>(2, 0) = 1.0;

        // RCLCPP_INFO(this->get_logger(), " %lf %lf %lf", cvCur.at<double>(0, 0), cvCur.at<double>(1, 0), cvCur.at<double>(2, 0));
        cvRef = cvCameraMatIv * cvRef;
        cvCur = cvCameraMatIv * cvCur;

        // RCLCPP_INFO(this->get_logger(), " %lf %lf %lf", cvCur.at<double>(0, 0), cvCur.at<double>(1, 0), cvCur.at<double>(2, 0));

        cv::Point2f pt2Ref(cvRef.at<double>(0, 0), cvRef.at<double>(1, 0));
        cv::Point2f pt2Cur(cvCur.at<double>(0, 0), cvCur.at<double>(1, 0));

        // RCLCPP_INFO(this->get_logger(), "ptRef : %lf   %lf", pt2Ref.x, pt2Ref.y);
        // RCLCPP_INFO(this->get_logger(), "ptCur : %lf   %lf", pt2Cur.x, pt2Cur.y);
        vRefKpLeftNorm.push_back(pt2Ref);
        vCurKpLeftNorm.push_back(pt2Cur);
    }

    cv::triangulatePoints(cvProjId, cvProjP2C, vRefKpLeftNorm, vCurKpLeftNorm, cvMonoDep4D);
    // cv::triangulatePoints(cvProjId,cvProjP2C,vRefKpLeftNorm,vCurKpLeftNorm,vMonoDep4D);

    // RCLCPP_INFO(this->get_logger(), "rowcol : %d  %d ",cvMonoDep4D.rows, cvMonoDep4D.cols);

    for (int i = 0; i < nVecSize; i++)
    {
        // RCLCPP_INFO(this->get_logger(), "4d : %lf %lf %lf %lf",
        // cvMonoDep4D.at<double>(0,i), cvMonoDep4D.at<double>(1,i), cvMonoDep4D.at<double>(2,i), cvMonoDep4D.at<double>(3,i) );
    }

    std::vector<cv::Point2f> vCurKpRight;
    std::vector<uchar> vCurStatus;
    cv::calcOpticalFlowPyrLK(cvLeftImage, cvRightImage, vCurKpLeftTracked, vCurKpRight, vCurStatus, cv::noArray(), cv::Size(21, 21), 3);

    // int nVecSize = static_cast<int>(vCurStatus.size());
    // double dSumScale = 0.0;
    int nInlierCnt = 0;

    double dScaleArr[1000];
    for (int i = 0; i < nVecSize; i++)
    {
        if (vCurStatus[i])
        { // if tracked;
            double dLeftX = vCurKpLeftTracked[i].x;
            // double dLeftY = vCurKpLeftTracked[i].y;

            double dRightX = vCurKpRight[i].x;
            // double dRightY = vCurKpRight[i].y;
            // RCLCPP_INFO(this->get_logger(), "X : %lf   %lf", dLeftX, dRightX);
            if (dLeftX - dRightX < 1)
                continue; // skip outlier with too small disparity

            // RCLCPP_INFO(this->get_logger(), "mono : %lf   %lf", cvMonoDep4D.at<float>(2,i), cvMonoDep4D.at<float>(3,i));

            float fMonoDep = cvMonoDep4D.at<float>(2, i) / cvMonoDep4D.at<float>(3, i);
            // double dMonoDep = vMonoDep4D[i][2] / vMonoDep4D[i][3];

            if (fMonoDep > 0.1 && fMonoDep < 100)
            {

                double dStereoDep = (dBaseline * dFx) / (dLeftX - dRightX);

                // RCLCPP_INFO(this->get_logger(), "dep : %lf   %lf", dMonoDep, dStereoDep);
                // dSumScale += dStereoDep / static_cast<double>(fMonoDep);
                // nInlierCnt++;
                dScaleArr[nInlierCnt++] = dStereoDep / static_cast<double>(fMonoDep);
            }
        }
    }

    // remove outlier
    std::sort(dScaleArr, dScaleArr + nInlierCnt);

    double dQ1 = dScaleArr[static_cast<int>(nInlierCnt / 4)];
    double dQ3 = dScaleArr[static_cast<int>(nInlierCnt / 4 * 3)];
    double dIqr = dQ3 - dQ1;
    double dLower = dQ1 - (dIqr * 1.5);
    double dUpper = dQ3 + (dIqr * 1.5);

    double dSumScale = 0.0;
    double nInlierCnt2 = 0;
    for (int i = 0; i < nInlierCnt; i++)
    {
        if (dScaleArr[i] > dLower && dScaleArr[i] < dUpper)
        {
            nInlierCnt2++;
            dSumScale += dScaleArr[i];
        }
    }

    dScale = dSumScale / nInlierCnt2;

    // dScale = dSumScale / static_cast<double>(nInlierCnt); // use mean val
    //  if(nInlierCnt < 500){
    //      RCLCPP_INFO(this->get_logger(), "warning  %d", nInlierCnt);
    //  }

    RCLCPP_INFO(this->get_logger(), "scale : %lf", dScale);
    return dScale;
    // return 1.0;
}

void ImagePublisher::findRT(cv::Mat &cvRotMat, cv::Mat &cvTransMat, std::vector<cv::Point2f> &vRefKpLeftTracked, std::vector<cv::Point2f> &vCurKpLeftTracked)
{

    // later consider distortion
    seok::TimeChecker tFindTime;
    cv::Mat cvEsMat;
    cvEsMat = cv::findEssentialMat(vCurKpLeftTracked, vRefKpLeftTracked, cvCameraMat, cv::RANSAC, 0.999, dRansacThresh);
    tFindTime.interval("Find Essen");
    seok::TimeChecker tReTime;
    cv::recoverPose(cvEsMat, vCurKpLeftTracked, vRefKpLeftTracked, cvCameraMat, cvRotMat, cvTransMat);
    tReTime.interval("recover");
}

void ImagePublisher::detectFeature(cv::Mat &cvImage, std::vector<cv::Point2f> &vKp)
{

    if (bUseFAST)
    {
        std::vector<cv::KeyPoint> vKpTmp;
        detector_->detect(cvImage, vKpTmp);
        vKp.clear();
        for (auto &pts : vKpTmp)
        {
            vKp.push_back(pts.pt);
        }
    }
    else if (bUseGFTT)
    {
        cv::Mat mask = cv::Mat(cvImage.rows, cvImage.cols, CV_8UC1, cv::Scalar(255));

        // vKp.clear();
        cv::goodFeaturesToTrack(cvImage, vKp, nMaxFeatureNumGFTT - vKp.size(), dQualityLev, nMinDist, mask); //, 3, false, 0.04)
    }
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
            vRefKpLeftTracked.push_back(vRefKpLeft[i]);
            vCurKpLeftTracked.push_back(vCurKpLeftTmp[i]);
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

void ImagePublisher::visualizePath()
{
    geometry_msgs::msg::PoseStamped poseStamp;

    poseStamp.pose.position.x = cvCurT.at<double>(0, 0);
    poseStamp.pose.position.y = cvCurT.at<double>(1, 0);
    poseStamp.pose.position.z = cvCurT.at<double>(2, 0);

    msgPath.poses.push_back(poseStamp);

    pathPublisher_->publish(msgPath);
}

void ImagePublisher::img1_callback(const sensor_msgs::msg::Image::SharedPtr imgMsg_)
{
    // RCLCPP_INFO(this->get_logger(), "img1 subscribed");
    left_image_buf.push(imgMsg_);
}

void ImagePublisher::img2_callback(const sensor_msgs::msg::Image::SharedPtr imgMsg_)
{
    //RCLCPP_INFO(this->get_logger(), "img2 subscribed");
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
        img_r_path = "/home/csw/dataset/KITTI/sequence/00/image_1/00" + std::to_string(nFrameCount) + ".png";
    }

    cvLeftImage = cv::imread(img_l_path, cv::IMREAD_GRAYSCALE);
    cvRightImage = cv::imread(img_r_path, cv::IMREAD_GRAYSCALE);
}

void ImagePublisher::readKittiGT()
{
    std::string strGT;
    std::getline(fsGroundTruth, strGT);

    std::vector<std::string> vecToken;
    std::istringstream issGT(strGT);
    std::string strBuffer;
    vecToken.clear();
    while (getline(issGT, strBuffer, ' '))
    {
        vecToken.push_back(strBuffer);
        // RCLCPP_INFO(this->get_logger(), "%s",strBuffer.c_str());
    }

    if (true)
    {
        geometry_msgs::msg::PoseStamped poseStamp;
        poseStamp.pose.position.x = std::stod(vecToken[3].c_str());
        poseStamp.pose.position.y = std::stod(vecToken[7].c_str());
        poseStamp.pose.position.z = std::stod(vecToken[11].c_str());

        msgGtPath.poses.push_back(poseStamp);

        gtPathPublisher_->publish(msgGtPath);
    }
}