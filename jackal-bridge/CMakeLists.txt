cmake_minimum_required(VERSION 3.8)
project(jackal_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(node_plugins "")

# Include directories
include_directories(
  include
)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME}_ros SHARED
  src/bridge_sender.cpp
)

target_link_libraries(${PROJECT_NAME}_ros
  rclcpp::rclcpp
  ${ZMQ_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_ros 
  rclcpp
  rclcpp_components
  geometry_msgs
)

add_executable(${PROJECT_NAME}_node
  src/bridge_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  rclcpp::rclcpp
  ${PROJECT_NAME}_ros
)

rclcpp_components_register_nodes(${PROJECT_NAME}_ros "JackalBridge::JackalBridgeSender")
set(node_plugins "${node_plugins}JackalBridge::JackalBridgeSender;$<TARGET_FILE:${PROJECT_NAME}_ros>\n")

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}_ros
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME}_ros)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  geometry_msgs
)

ament_package()


