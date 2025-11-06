/*!
* @Author Jason Hughes
* @Date October2025
*
* @Brief: Connect to jackal serial port
* and publish drive commands to the jackal
* and get feedback from the board.
*/

#include "jackal_serial/ros/jackal_serial_node.hpp" 

using namespace JackalSerial;

JackalSerialNode::JackalSerialNode(const rclcpp::NodeOptions& options) : Node("jackal_serial_node", options)
{
    declare_parameter<int>("baud_rate", 115200);
    declare_parameter<std::string>("dev", "/dev/jackal");

    int baud_rate;
    get_parameter("baud_rate", baud_rate);
    std::string dev;
    get_parameter("dev", dev);

    jackal_ = std::make_unique<SerialCore>(dev, baud_rate);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&JackalSerialNode::jointStateCallback, this, std::placeholders::_1));

    initializeTimeSync();
}

void JackalSerialNode::initializeTimeSync()
{
    rclcpp::Time now = this->now();
    std::pair<uint32_t, uint32_t> time = Conversions::convertTime(now);
    jackal_->writeTime(time);
}

void JackalSerialNode::jointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
    Drive drive = Conversions::convertJointStates(msg);
    jackal_->writeDriveMsg(drive);
}

RCLCPP_COMPONENTS_REGISTER_NODE(JackalSerial::JackalSerialNode)
