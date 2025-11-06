/*!
* @Author Jason Hughes
* @Date October 2025 
*
* @About start the zmq sender node
*/

#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>

#include "jackal_bridge/bridge_sender.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    try {
        auto node = std::make_shared<JackalBridge::JackalBridgeSender>(options);
        rclcpp::executors::MultiThreadedExecutor executor;

        executor.add_node(node);
        executor.spin();

        rclcpp::shutdown();
    }
    catch (const std::exception& e) {
        std::cerr << "[JackalBridgeSender] Caugh Error: " << e.what() << std::endl;
    }

    return 0;
}
