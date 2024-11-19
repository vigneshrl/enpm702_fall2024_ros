cmake_minimum_required(VERSION 3.8)
project(interface_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(enpm702_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include/interface_demo)

add_executable(interface_demo src/main.cpp src/interface_demo_node.cpp src/color_utils.cpp)

ament_target_dependencies(interface_demo rclcpp enpm702_msgs nav_msgs tf2_geometry_msgs)

install(TARGETS interface_demo DESTINATION lib/${PROJECT_NAME})


ament_package()
