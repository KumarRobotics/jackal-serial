cmake_minimum_required(VERSION 3.8)
project(jackal_serial)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_TESTS "build test scripts" OFF)

if(BUILD_TESTS)
    message("Build Tests is enabled")
    add_definitions(-DBUILD_TESTS)
else()
    message("Build Tests is disabled")
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)
find_package(GTest REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

set(node_plugins "")

include_directories(
  include
  ${LIBSERIAL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/drive.cpp
  src/serial.cpp
  src/packets.cpp
  src/serializer.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${LIBSERIAL_LIBRARIES}
)

add_library(${PROJECT_NAME}_ros SHARED
  ros/conversions.cpp
  ros/jackal_serial_node.cpp
)

target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_ros 
  rclcpp
  rclcpp_components
  sensor_msgs
)

add_executable(${PROJECT_NAME}_node
  ros/node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  rclcpp::rclcpp
  ${PROJECT_NAME}
  ${PROJECT_NAME}_ros
)

rclcpp_components_register_nodes(${PROJECT_NAME}_ros "JackalSerial::JackalSerialNode")
set(node_plugins "${node_plugins}JackalSerial::JackalSerialNode;$<TARGET_FILE:${PROJECT_NAME}_ros>\n")

install(DIRECTORY include/
  DESTINATION include/
)

# Install launch and config files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_ros
  ${PROJECT_NAME}_node
  EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  ${PROJECT_NAME}
  ${PROJECT_NAME}_ros
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if (BUILD_TESTS)
  enable_testing()

  add_executable(rcl_conversion_test test/test_conversions.cpp)
  target_link_libraries(rcl_conversion_test
    GTest::GTest
    GTest::Main
    ${PROJECT_NAME}
    ${PROJECT_NAME}_ros
  )

  add_executable(serialization_test test/test_serialization.cpp)
  target_link_libraries(serialization_test
    GTest::GTest
    GTest::Main
    ${PROJECT_NAME}
    ${PROJECT_NAME}_ros
  )

  add_executable(packet_test test/test_packet.cpp)
  target_link_libraries(packet_test
    GTest::GTest
    GTest::Main
    ${PROJECT_NAME}
    ${PROJECT_NAME}_ros
  )

  include(GoogleTest)
  gtest_discover_tests(rcl_conversion_test) 
  gtest_discover_tests(serialization_test)
  gtest_discover_tests(packet_test)
endif()

ament_package()
