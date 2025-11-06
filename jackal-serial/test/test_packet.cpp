/*!
* @Author: Jason Hughes
* @Date: November 2025
*
* @Brief: test send a time
* packet to the jackal
*/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "jackal_serial/ros/conversions.hpp"
#include "jackal_serial/core/serializer.hpp"
#include "jackal_serial/core/packets.hpp"


TEST(PacketTestSuite, PacketHeader)
{
    rclcpp::Time ros_time = rclcpp::Clock().now();
    std::pair<uint32_t, uint32_t> my_time = JackalSerial::Conversions::convertTime(ros_time);
    std::vector<uint8_t> serialized_msg = JackalSerial::JackalSerializer::serializeTimeMsg(my_time.first, my_time.second);
    std::vector<uint8_t> packet = JackalSerial::JackalPacket::getSerialPacket(JackalSerial::TopicID::TIMESYNC, serialized_msg);
    
    ASSERT_EQ(packet[0], 0xFF);
    ASSERT_EQ(packet[1], 0xFE);
}

TEST(PacketTestSuite, PacketSize)
{
    rclcpp::Time ros_time = rclcpp::Clock().now();
    std::pair<uint32_t, uint32_t> my_time = JackalSerial::Conversions::convertTime(ros_time);
    std::vector<uint8_t> serialized_msg = JackalSerial::JackalSerializer::serializeTimeMsg(my_time.first, my_time.second);
    std::vector<uint8_t> packet = JackalSerial::JackalPacket::getSerialPacket(JackalSerial::TopicID::TIMESYNC, serialized_msg);
    
    ASSERT_EQ(packet.size(), 16);
}

TEST(PacketTestSuite, PacketSizeCheckSum)
{ 
    rclcpp::Time ros_time = rclcpp::Clock().now();
    std::pair<uint32_t, uint32_t> my_time = JackalSerial::Conversions::convertTime(ros_time);
    std::vector<uint8_t> serialized_msg = JackalSerial::JackalSerializer::serializeTimeMsg(my_time.first, my_time.second);
    std::vector<uint8_t> packet = JackalSerial::JackalPacket::getSerialPacket(JackalSerial::TopicID::TIMESYNC, serialized_msg);

    ASSERT_EQ(packet[4], 0xF7);
}
