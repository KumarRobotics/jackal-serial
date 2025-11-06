/*!
 * @Author Jason Hughes
 * @Date November 2025
 *
 * @Brief start the jackal serial
 * connection as a ros2 node
 */

#include <rclcpp/rclcpp.hpp>

#include "jackal_serial/ros/jackal_serial_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    try
    {
        auto node = std::make_shared<JackalSerial::JackalSerialNode>(options);
        rclcpp::executors::SingleThreadedExecutor executor;

        executor.add_node(node);

        executor.spin();

        rclcpp::shutdown();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[JACKAL-SERIAL] Caught error " << e.what() << std::endl;
    }
    return 0;
}

