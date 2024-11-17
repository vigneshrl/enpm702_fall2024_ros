cmake_minimum_required(VERSION 3.8)
project(qos_publisher_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(include/qos_publisher_demo)

add_executable(qos_publisher_demo src/main.cpp src/qos_publisher_demo_node.cpp)

ament_target_dependencies(qos_publisher_demo rclcpp example_interfaces)

install(TARGETS qos_publisher_demo DESTINATION lib/${PROJECT_NAME})


ament_package()
