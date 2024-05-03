#include "monocular-inertial-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera", 10, std::bind(&MonocularInertialNode::GrabImage, this, _1));

    subImu_ = this->create_subscription<ImuMsg>(
        "imu", 1000, std::bind(&MonocularInertialNode::GrabImu, this, _1));

    syncThread_ = new std::thread(&MonocularInertialNode::SyncWithImu, this);

    std::cout << "System Initialization Complete" << std::endl;
}

MonocularInertialNode::~MonocularInertialNode()
{
//    if (syncThread_->joinable()) {
    syncThread_->join();
//    }
    delete syncThread_;

    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    if (!std::isnan(msg->linear_acceleration.x) && !std::isnan(msg->linear_acceleration.y) &&
        !std::isnan(msg->linear_acceleration.z) && !std::isnan(msg->angular_velocity.x) &&
        !std::isnan(msg->angular_velocity.y) && !std::isnan(msg->angular_velocity.z))
    {
        bufMutex_.lock();
        imuBuf_.push(msg);
        bufMutex_.unlock();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid IMU data");
    }
}

void MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    bufMutexImg_.lock();

    if (!imgBuf_.empty())
        imgBuf_.pop();
    imgBuf_.push(msg);

    bufMutexImg_.unlock();
}

cv::Mat MonocularInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonocularInertialNode::SyncWithImu()
{
//    const double maxTimeDiff = 0.25;  // Maximum allowed time difference for synchronization

while (1)
    {
        cv::Mat imageFrame;
        double tImage = 0;
        if (!imgBuf_.empty() && !imuBuf_.empty())
        {
            tImage = Utility::StampToSec(imgBuf_.front()->header.stamp);

            bufMutexImg_.lock();
            imageFrame = GetImage(imgBuf_.front());
            imgBuf_.pop();
            bufMutexImg_.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            bufMutex_.lock();
            if (!imuBuf_.empty())
            {
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImage)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutex_.unlock();

            if (vImuMeas.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Empty IMU vector encountered. Skipping frame.");
                continue; // Skip processing this frame
            }

            try
            {
                m_SLAM->TrackMonocular(imageFrame, tImage, vImuMeas);
                std::cout<<"one frame has been sent"<<std::endl;
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "ORB_SLAM3 node processing error: %s", e.what())
            }

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}
