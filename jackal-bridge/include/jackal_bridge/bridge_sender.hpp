/*
* @Author Jason Hughes
* @Date October 2025
*
* @brief a node to convert a ros2 Twist msg to
* a zmq msg to send to ros1 
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <zmq.hpp>

namespace JackalBridge
{

struct TwistZMQ
{
    double lx, ly, lz, ax, ay, az;
};

class JackalBridgeSender : public rclcpp::Node
{
    public:
        JackalBridgeSender() = default;
        JackalBridgeSender(const rclcpp::NodeOptions& options);

    private:
        // callback
        void twistCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg);

        //subscriber
        rclcpp::Subscription<geometry_msgs::msg::Twist>::ConstSharedPtr vel_sub_;

        //zmq socket
        zmq::socket_t socket_;
        zmq::context_t context_;
};
}
