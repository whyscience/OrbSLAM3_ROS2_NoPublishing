#include "monocular-inertial-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularInertialNode::MonocularInertialNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera/image_mono", 10, std::bind(&MonocularInertialNode::GrabImage, this, _1));

    subImu_ = this->create_subscription<ImuMsg>(
        "imu", 1000, std::bind(&MonocularInertialNode::GrabImu, this, _1));

    syncThread_ = new std::thread(&MonocularInertialNode::SyncWithImu, this);

    std::cout << "System Initialization Complete" << std::endl;
}

MonocularInertialNode::~MonocularInertialNode()
{
    if (syncThread_->joinable()) {
        syncThread_->join();
    }
    delete syncThread_;

    m_SLAM->Shutdown();
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_Monocular_Inertial.txt");
}

void MonocularInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(bufMutex_);
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void MonocularInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(bufMutex_);
    imgBuf_.push(msg);
    bufMutex_.unlock();
}

void MonocularInertialNode::SyncWithImu()
{
    while (rclcpp::ok()) {
        std::unique_lock<std::mutex> lock(bufMutex_);
        if (!imgBuf_.empty() && !imuBuf_.empty()) {
            auto img = imgBuf_.front();
            auto img_time = img->header.stamp;

            // Find the closest IMU messages around the image timestamp
            while (!imuBuf_.empty() && (imuBuf_.front()->header.stamp < img_time)) {
                imuQueue_.push_back(imuBuf_.front());
                imuBuf_.pop();
            }

            // Check if IMU data is sufficiently close to image data
            if (!imuQueue_.empty() && isTimeClose(imuQueue_.back()->header.stamp, img_time)) {
                // Process image and IMU data here
                processFrame(img, imuQueue_);
                imuQueue_.clear();
                imgBuf_.pop();
            }
        }
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool MonocularInertialNode::isTimeClose(const rclcpp::Time& t1, const rclcpp::Time& t2, double threshold = 0.05)
{
    return std::abs((t1 - t2).seconds()) < threshold;
}

void MonocularInertialNode::processFrame(const ImageMsg::SharedPtr& img, const std::vector<ImuMsg::SharedPtr>& imuMsgs)
{
    cv::Mat cvImage = cv_bridge::toCvShare(img, "bgr8")->image;

    // Convert IMU data to the format expected by ORB_SLAM3
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    for (const auto& imu : imuMsgs) {
        double t = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;
        cv::Point3f acc(imu->linear_acceleration.x,
                        imu->linear_acceleration.y,
                        imu->linear_acceleration.z);
        cv::Point3f gyr(imu->angular_velocity.x,
                        imu->angular_velocity.y,
                        imu->angular_velocity.z);
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
    }

    // Pass to SLAM system
    m_SLAM->TrackMonocular(cvImage, img->header.stamp.sec + img->header.stamp.nanosec * 1e-9, vImuMeas);
}