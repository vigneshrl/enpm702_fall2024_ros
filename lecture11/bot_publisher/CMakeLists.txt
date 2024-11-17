cmake_minimum_required(VERSION 3.8)
project(bot_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(include/bot_publisher)

add_executable(bot_pub src/main.cpp src/bot_publisher.cpp)

ament_target_dependencies(bot_pub rclcpp geometry_msgs)

install(TARGETS bot_pub DESTINATION lib/${PROJECT_NAME})

ament_package()
