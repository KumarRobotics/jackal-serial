
/*!
* @Author: Jason Hughes
* @Date: November 2025
*
* @Brief: test sending time packet to jackal
*/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "jackal_serial/ros/conversions.hpp"
#include "jackal_serial/core/serial.hpp"


TEST(SerialConnectionTestSuite, SendTime)
{ 
    rclcpp::Time ros_time = rclcpp::Clock().now();
    std::pair<uint32_t, uint32_t> my_time = JackalSerial::Conversions::convertTime(ros_time);

    std::string dev = "/dev/jackal";
    JackalSerial::SerialCore jackal(dev, 115200);

    jackal.writeTime(my_time);
}
