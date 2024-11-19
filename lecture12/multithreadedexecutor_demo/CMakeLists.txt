cmake_minimum_required(VERSION 3.8)
project(multithreadedexecutor_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include/multithreadedexecutor_demo)

add_executable(multithreadedexecutor_demo src/main.cpp src/multithreadedexecutor_demo_node.cpp src/color_utils.cpp)

ament_target_dependencies(multithreadedexecutor_demo rclcpp std_msgs)

install(TARGETS multithreadedexecutor_demo DESTINATION lib/${PROJECT_NAME})


ament_package()
