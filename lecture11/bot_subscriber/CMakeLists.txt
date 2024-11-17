cmake_minimum_required(VERSION 3.8)
project(bot_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include/bot_subscriber)

add_executable(bot_sub src/main.cpp src/bot_subscriber.cpp)

ament_target_dependencies(bot_sub rclcpp nav_msgs sensor_msgs)

install(TARGETS bot_sub DESTINATION lib/${PROJECT_NAME})
ament_package()
