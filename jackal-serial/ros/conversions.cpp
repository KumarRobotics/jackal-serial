
/*!
* @Author: Jason Hughes
* @Date: November 2025
*
* @Brief: convert ros2 msgs to
* regular data types
*/

#include "jackal_serial/ros/conversions.hpp"

std::pair<uint32_t, uint32_t> JackalSerial::Conversions::convertTime(rclcpp::Time& time)
{
    uint32_t sec = time.seconds();
    uint32_t nsec = time.nanoseconds() % 1000000000;

    return std::pair<uint32_t, uint32_t>(sec, nsec);
}

JackalSerial::Drive JackalSerial::Conversions::convertJointStates(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
    // average two left and right velocities
    // and cast from double to float
    float left_vel = static_cast<float>((msg->velocity[0] + msg->velocity[2]) / 2.0);
    float right_vel = static_cast<float>((msg->velocity[1] + msg->velocity[3]) / 2.0);

    JackalSerial::Drive drive(0, left_vel, right_vel);

    return drive;    
}
