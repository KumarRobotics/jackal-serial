#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "serial_driver/serial_driver.hpp"
#include "io_context/io_context.hpp"
#include <filesystem>
#include <thread>
#include <chrono>
#include <csignal>
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <regex>

#define DEVICEKEY "Teensyduino"

namespace jackal_teleop
{

class JetiJoy : public rclcpp::Node
{
public:
  explicit JetiJoy(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
  : Node("jeti_joy", options),
    running_(true)
  {
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);

    io_context_ = std::make_unique<drivers::common::IoContext>();

    signal(SIGINT, signal_handler);
    worker_thread_ = std::thread(&JetiJoy::loop, this);
  }

  ~JetiJoy()
  {
    running_ = false;
    if (serial_port_ && serial_port_->is_open()) {
      serial_port_->close();
    }
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  static void signal_handler(int)
  {
    std::cout << "\n[Jeti-Joy] Shutting down..." << std::endl;
    rclcpp::shutdown();
  }

  std::string find_device()
  {
    namespace fs = std::filesystem;
    std::string base_path = "/dev/serial/by-id";

    while (rclcpp::ok() && running_) {
      try {
        for (auto &entry : fs::directory_iterator(base_path)) {
          std::string name = entry.path().string();
          if (name.find(DEVICEKEY) != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "Found device: %s", name.c_str());
            return name;
          }
        }
        RCLCPP_INFO(this->get_logger(), "Device not found, retrying eventually .....");
        std::this_thread::sleep_for(std::chrono::seconds(2));
      } catch (const fs::filesystem_error &e) {
        RCLCPP_ERROR(this->get_logger(), "Error accessing %s: %s", base_path.c_str(), e.what());
        std::this_thread::sleep_for(std::chrono::seconds(2));
      }
    }
    return "";
  }

  bool connect_to_device()
  {
    try {
      if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
      }

      std::string dev = find_device();
      if (dev.empty()) {
        return false;
      }

      using drivers::serial_driver::FlowControl;
      using drivers::serial_driver::Parity;
      using drivers::serial_driver::StopBits;
      using drivers::serial_driver::SerialPortConfig;

      SerialPortConfig config(115200, FlowControl::NONE, Parity::NONE, StopBits::ONE);

      serial_port_ = std::make_unique<drivers::serial_driver::SerialPort>(
          *io_context_, dev, config);
      serial_port_->open();

      if (serial_port_->is_open()) {
        RCLCPP_INFO(this->get_logger(), "Connected to serial device: %s", dev.c_str());
        return true;
      }
      return false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect: %s", e.what());
      return false;
    }
  }

  void loop()
  {
    if (!connect_to_device()) {
      RCLCPP_ERROR(this->get_logger(), "Failed initial connection.");
      return;
    }

    int consecutive_errors = 0;
    const int max_errors = 5;
    buffer_.resize(256); // Preallocate buffer once

    while (rclcpp::ok() && running_) {
      try {
        if (!serial_port_ || !serial_port_->is_open()) {
          RCLCPP_WARN(this->get_logger(), "Serial port closed, attempting reconnect...");
          if (!connect_to_device()) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
          }
        }

        size_t bytes_read = serial_port_->receive(buffer_);
        if (bytes_read > 0) {
          consecutive_errors = 0;
          std::string data(buffer_.begin(), buffer_.begin() + bytes_read);
          process_data(data);
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

      } catch (const std::exception &e) {
        consecutive_errors++;
        RCLCPP_ERROR(this->get_logger(), "Serial error (%d/%d): %s",
                     consecutive_errors, max_errors, e.what());

        if (consecutive_errors >= max_errors) {
          RCLCPP_WARN(this->get_logger(), "Too many errors, reconnecting...");
          consecutive_errors = 0;

          try {
            if (serial_port_ && serial_port_->is_open())
              serial_port_->close();
          } catch (...) {}

          std::this_thread::sleep_for(std::chrono::seconds(2));
          connect_to_device();
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    }

    if (serial_port_ && serial_port_->is_open()) {
      serial_port_->close();
    }
  }

  void process_data(const std::string &raw)
  {
    static const std::regex pattern(
      R"(Throttle \(CH1\): ([\-0-9.]+)\s+Aileron \(CH2\): ([\-0-9.]+)\s+Streamer \(CH3\): (\d+)\s+Trigger \(CH5\): (\d+)\s+E-Stop \(CH6\): (\d+))"
    );

    std::smatch match;

    if (std::regex_search(raw, match, pattern)) {
      try {
        float throttle = std::stof(match[1].str());
        float aileron = std::stof(match[2].str());
        int streamer = std::stoi(match[3].str());
        int trigger = std::stoi(match[4].str());
        int estop = std::stoi(match[5].str());

        auto msg = sensor_msgs::msg::Joy();
        msg.axes = {throttle, aileron, 0.0f, 0.0f};
        msg.buttons = {trigger, estop, streamer, 0};

        joy_pub_->publish(msg);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "[JETI-JOY] Error parsing floats: %s", e.what());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "[JETI-JOY] No regex match for line: %s", raw.c_str());
    }
  }


  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  std::unique_ptr<drivers::serial_driver::SerialPort> serial_port_;
  std::unique_ptr<drivers::common::IoContext> io_context_;
  std::thread worker_thread_;
  std::vector<uint8_t> buffer_;
  std::atomic<bool> running_;
};

} 

RCLCPP_COMPONENTS_REGISTER_NODE(jackal_teleop::JetiJoy)


