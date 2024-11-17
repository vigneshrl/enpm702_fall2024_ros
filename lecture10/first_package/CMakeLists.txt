cmake_minimum_required(VERSION 3.8)
project(first_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include/first_package)

add_executable(hello src/main.cpp src/first_node.cpp)

ament_target_dependencies(hello rclcpp)

install(TARGETS hello DESTINATION lib/${PROJECT_NAME})

ament_package()
