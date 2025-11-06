/*!
* @Author Jason Hughes
* @Date November 2025
*
* @Brief test the conversion from
* ros2 structures to cpp structures
*/

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "jackal_serial/ros/conversions.hpp"

TEST(ConversionTestSuite, TestTimeConversion)
{
    rclcpp::Time ros_time = rclcpp::Clock().now();
    std::pair<uint32_t, uint32_t> my_time = JackalSerial::Conversions::convertTime(ros_time);

    ASSERT_EQ(static_cast<uint32_t>(ros_time.seconds()), my_time.first);
    ASSERT_EQ(static_cast<uint32_t>(ros_time.nanoseconds()%1000000000), my_time.second);
}
