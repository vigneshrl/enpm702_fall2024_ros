cmake_minimum_required(VERSION 3.8)
project(qos_demo_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(include/qos_demo_publisher)

add_executable(qos_demo_publisher src/main.cpp src/qos_demo_publisher_node.cpp)

ament_target_dependencies(qos_demo_publisher rclcpp example_interfaces)

install(TARGETS qos_demo_publisher DESTINATION lib/${PROJECT_NAME})


ament_package()
