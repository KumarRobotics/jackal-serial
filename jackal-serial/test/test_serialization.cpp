
/*!
* @Author: Jason Hughes
* @Date: November 2025
*
* @Brief: test that the msg serialization
* makes sense
*/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "jackal_serial/ros/conversions.hpp"
#include "jackal_serial/core/serializer.hpp"

TEST(SerializationTestSuite, TimeSerialization)
{
    rclcpp::Time ros_time = rclcpp::Clock().now();
    std::pair<uint32_t, uint32_t> my_time = JackalSerial::Conversions::convertTime(ros_time);
    std::vector<uint8_t> serialized_msg = JackalSerial::JackalSerializer::serializeTimeMsg(my_time.first, my_time.second);

    ASSERT_EQ(serialized_msg.size(), 8);
}

TEST(SerializationTestSuite, DriveSerialization)
{ 
    JackalSerial::Drive drive(0, 1.0, 1.0);
    std::vector<uint8_t> serialized_msg = JackalSerial::JackalSerializer::serializeDriveMsg(drive);

    ASSERT_EQ(serialized_msg.size(), 9);
}

