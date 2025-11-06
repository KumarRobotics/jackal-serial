#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>

namespace jackal_teleop
{

class JackalTeleop : public rclcpp::Node
{
public:
  explicit JackalTeleop(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("jackal_teleop", options),
    is_auto_(false),
    is_streaming_(false),
    trigger_(false),
    prev_trigger_(true),
    trigger_count_(0),
    num_views_(2)
  {
    twist_buf_ = geometry_msgs::msg::TwistStamped();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JackalTeleop::joy_cb, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/jackal_velocity_controller/cmd_vel", 10);
    is_auto_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_auto", 10);
    trigger_pub_ = this->create_publisher<std_msgs::msg::UInt8>("trigger", 10);
    is_streaming_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_streaming", 10);

    pub_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&JackalTeleop::pub_cb, this));
  }

private:
  float apply_deadzone(float value, float threshold = 0.04)
  {
    return (std::abs(value) < threshold) ? 0.0f : value;
  }

  void pub_cb()
  {
    if (!is_auto_) {
      twist_pub_->publish(twist_buf_);
    }
  }

  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    twist_buf_.twist.linear.x = apply_deadzone(msg->axes[0]) * 2.0;
    twist_buf_.twist.angular.z = 1.5 * apply_deadzone(msg->axes[1]) * 0.5;

    is_auto_ = msg->buttons[1];
    trigger_ = msg->buttons[0];
    is_streaming_ = msg->buttons[2];

    std_msgs::msg::Bool is_auto_msg;
    is_auto_msg.data = is_auto_;
    is_auto_pub_->publish(is_auto_msg);

    std_msgs::msg::Bool is_streaming_msg;
    is_streaming_msg.data = is_streaming_;
    is_streaming_pub_->publish(is_streaming_msg);

    if (is_auto_) {
      trigger_ = 0;
      return;
    }

    if (trigger_ == 1 && prev_trigger_ == 0) {
      std_msgs::msg::UInt8 trigger_msg;
      trigger_msg.data = trigger_count_;
      trigger_pub_->publish(trigger_msg);

      trigger_count_ = (trigger_count_ + 1) % num_views_;
    }

    prev_trigger_ = trigger_;
  }

  geometry_msgs::msg::TwistStamped twist_buf_;
  bool is_auto_;
  bool is_streaming_;
  bool trigger_;
  bool prev_trigger_;
  uint8_t trigger_count_;
  uint8_t num_views_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_auto_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_streaming_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

}  


RCLCPP_COMPONENTS_REGISTER_NODE(jackal_teleop::JackalTeleop)

