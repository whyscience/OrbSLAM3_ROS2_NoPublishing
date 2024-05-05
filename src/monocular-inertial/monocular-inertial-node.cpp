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
        RCLCPP_ERROR(this->get_logger(), "Invalid IMU data - Rxd NaN");
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

void MonocularInertialNode::SyncWithImu() {
    while (rclcpp::ok()) {
        std::unique_lock<std::mutex> img_lock(bufMutexImg_, std::defer_lock);
        std::unique_lock<std::mutex> imu_lock(bufMutex_, std::defer_lock);

        std::lock(img_lock, imu_lock);  // Lock both mutexes without deadlock

        if (!imgBuf_.empty() && !imuBuf_.empty()) {
            auto imgPtr = imgBuf_.front();
            double tImage = imgPtr->header.stamp.sec + imgPtr->header.stamp.nanosec * 1e-9;

            // Collect all IMU data up to the timestamp of the image
            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            while (!imuBuf_.empty() && (imuBuf_.front()->header.stamp.sec + imuBuf_.front()->header.stamp.nanosec * 1e-9) <= tImage) {
                auto imuMsg = imuBuf_.front();
                imuBuf_.pop();
                ORB_SLAM3::IMU::Point imuPoint{
                    cv::Point3f(imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z),
                    cv::Point3f(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z),
                    imuMsg->header.stamp.sec + imuMsg->header.stamp.nanosec * 1e-9
                };
                vImuMeas.push_back(imuPoint);
            }

            if (!vImuMeas.empty()) {
                cv::Mat imageFrame = GetImage(imgPtr); // Assuming GetImage handles cv_bridge and ROS2 image message conversion
                imgBuf_.pop();
                m_SLAM->TrackMonocular(imageFrame, tImage, vImuMeas);
            } else {
                RCLCPP_WARN(this->get_logger(), "No corresponding IMU data for image at %f", tImage);
            }
        }

        img_lock.unlock();
        imu_lock.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


