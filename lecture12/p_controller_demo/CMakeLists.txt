cmake_minimum_required(VERSION 3.8)
project(p_controller_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(include/p_controller_demo)

add_executable(p_controller_demo src/main.cpp src/p_controller_demo_node.cpp src/color_utils.cpp)

ament_target_dependencies(p_controller_demo rclcpp nav_msgs geometry_msgs tf2_geometry_msgs)

install(TARGETS p_controller_demo DESTINATION lib/${PROJECT_NAME})


ament_package()
