cmake_minimum_required(VERSION 3.8)
project(quaternion_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include/quaternion_demo)

add_executable(quat_demo src/main.cpp src/quaternion_example.cpp)

ament_target_dependencies(quat_demo rclcpp geometry_msgs tf2)

install(TARGETS quat_demo DESTINATION lib/${PROJECT_NAME})

ament_package()
