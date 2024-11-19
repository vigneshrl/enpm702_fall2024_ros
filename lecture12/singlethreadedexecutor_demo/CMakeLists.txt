cmake_minimum_required(VERSION 3.8)
project(singlethreadedexecutor_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include/singlethreadedexecutor_demo)

add_executable(singlethreadedexecutor_demo src/main.cpp src/singlethreadedexecutor_demo_node.cpp src/color_utils.cpp)

ament_target_dependencies(singlethreadedexecutor_demo rclcpp)

install(TARGETS singlethreadedexecutor_demo DESTINATION lib/${PROJECT_NAME})


ament_package()
