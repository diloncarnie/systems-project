/**
* This file is part of R-VIO2.
*
* Copyright (C) 2022 Zheng Huai <zhuai@udel.edu> and Guoquan Huang <ghuang@udel.edu>
* For more information see <http://github.com/rpng/R-VIO2> 
*
* R-VIO2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* R-VIO2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with R-VIO2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "rvio2/System.h"

class ImageGrabber
{
public:
    ImageGrabber(RVIO2::System* pSys) : mpSys(pSys) {}

    // Callback now receives a ConstSharedPtr to a sensor_msgs::msg::Image
    void GrabImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        // Note: ROS2 no longer provides header.seq, so the sequence check has been removed.
        
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ImageGrabber"), "cv_bridge exception: %s", e.what());
            return;
        }

        RVIO2::ImageData* pData = new RVIO2::ImageData();
        pData->Image = cv_ptr->image.clone();
        // Convert ROS2 timestamp to seconds manually
        pData->Timestamp = cv_ptr->header.stamp.sec + cv_ptr->header.stamp.nanosec * 1e-9;

        mpSys->PushImageData(pData);
        mpSys->run();
    }

    RVIO2::System* mpSys;
};


class ImuGrabber
{
public:
    ImuGrabber(RVIO2::System* pSys) : mpSys(pSys) {}

    // Callback now receives a ConstSharedPtr to a sensor_msgs::msg::Imu
    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        // Note: The ROS2 header does not include a seq field, so the sequence check is removed.
        
        Eigen::Vector3f angular_velocity(msg->angular_velocity.x,
                                           msg->angular_velocity.y,
                                           msg->angular_velocity.z);
        Eigen::Vector3f linear_acceleration(msg->linear_acceleration.x,
                                            msg->linear_acceleration.y,
                                            msg->linear_acceleration.z);

        double currtime = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        RVIO2::ImuData* pData = new RVIO2::ImuData();
        pData->AngularVel = angular_velocity;
        pData->LinearAccel = linear_acceleration;
        pData->Timestamp = currtime;

        static double lasttime = -1;
        if (lasttime != -1)
            pData->TimeInterval = currtime - lasttime;
        else
            pData->TimeInterval = 0;
        lasttime = currtime;

        mpSys->PushImuData(pData);
    }

    RVIO2::System* mpSys;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create a ROS2 node
    auto node = rclcpp::Node::make_shared("rvio2_mono");

    // Create your system using the node so that it can retrieve its parameters.
    RVIO2::System Sys(node);
    RCLCPP_INFO(node->get_logger(), "Initialized System");

    ImageGrabber igb1(&Sys);
    ImuGrabber igb2(&Sys);


    // Create subscriptions with appropriate QoS settings.
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::QoS(100),
        std::bind(&ImuGrabber::GrabImu, &igb2, std::placeholders::_1)
    );

    auto image_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::QoS(1),
        std::bind(&ImageGrabber::GrabImage, &igb1, std::placeholders::_1)
    );

    // Spin the node so callbacks are processed
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
