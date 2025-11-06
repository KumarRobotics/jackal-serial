/*!
*
*
*/

#include "jackal_bridge/bridge_sender.hpp"

using namespace JackalBridge;

JackalBridgeSender::JackalBridgeSender(const rclcpp::NodeOptions& options) : rclcpp::Node("jackal_bridge", options)
{
    vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&JackalBridgeSender::twistCallback, this, std::placeholders::_1));
    
    context_ = zmq::context_t(1);
    socket_ = zmq::socket_t(context_, zmq::socket_type::push);

    socket_.bind("tcp://127.0.0.1:8281");
    RCLCPP_INFO(get_logger(), "[JackalBridgeSender] Initialized");
}

void JackalBridgeSender::twistCallback(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
    TwistZMQ data = {msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z};

    zmq::message_t message(sizeof(TwistZMQ));
    std::memcpy(message.data(), &data, sizeof(TwistZMQ));
    socket_.send(message, zmq::send_flags::none);
}

RCLCPP_COMPONENTS_REGISTER_NODE(JackalBridge::JackalBridgeSender)
