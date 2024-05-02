#ifndef __MONOCULAR_INERTIAL_NODE_HPP__
#define __MONOCULAR_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class MonocularInertialNode : public rclcpp::Node
{
public:
    MonocularInertialNode(ORB_SLAM3::System* pSLAM);
    ~MonocularSlamNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void SyncWithImu();

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;

    std::queue<ImuMsg::SharedPtr> imuBuf_;
    std::queue<ImageMsg::SharedPtr> imgBuf_;
    std::mutex bufMutex_;

    std::thread *syncThread_;
};

#endif
