/*!
* @Author: Jason Hughes
* @Date: November 2025
*
* @Brief: convert ros2 msgs to
* regular data types
*/

#pragma once

#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "jackal_serial/core/drive.hpp"

namespace JackalSerial
{

class Conversions 
{
    public:
        Conversions() = default;
        
        static std::pair<uint32_t,uint32_t> convertTime(rclcpp::Time& time);
	static Drive convertJointStates(const sensor_msgs::msg::JointState::ConstSharedPtr msg);        
};
}
