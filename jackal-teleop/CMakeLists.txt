cmake_minimum_required(VERSION 3.8)
project(jackal_teleop)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)

# Include directories
include_directories(include)

# JetiJoy Component 

add_library(jeti_joy SHARED src/jeti_joy.cpp)
ament_target_dependencies(jeti_joy
  rclcpp
  rclcpp_components
  sensor_msgs
  serial_driver
)
rclcpp_components_register_nodes(jeti_joy "jackal_teleop::JetiJoy")


# JackalTeleop Component 

add_library(jackal_teleop SHARED src/jackal_teleop.cpp)
ament_target_dependencies(jackal_teleop
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
)
rclcpp_components_register_nodes(jackal_teleop "jackal_teleop::JackalTeleop")


# Install 

install(TARGETS
  jeti_joy
  jackal_teleop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
