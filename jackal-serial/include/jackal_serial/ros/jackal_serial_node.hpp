/*!
* @Author Jason Hughes
* @Date October 2025
*
* @About the serial driver for the jackal board
*/

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "jackal_serial/core/serial.hpp"
#include "jackal_serial/ros/conversions.hpp"

namespace JackalSerial
{
class JackalSerialNode : public rclcpp::Node
{
    public: 
        JackalSerialNode() = default;
        JackalSerialNode(const rclcpp::NodeOptions& options);

    private:
	void initializeTimeSync();
	void jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
	
	rclcpp::Subscription<sensor_msgs::msg::JointState>::ConstSharedPtr js_sub_; 

	std::unique_ptr<SerialCore> jackal_; 
};
}

