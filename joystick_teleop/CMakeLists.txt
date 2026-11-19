cmake_minimum_required(VERSION 3.8)
project(joystick_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(joystick_teleop_node src/joystick_teleop.cpp)

ament_target_dependencies(joystick_teleop_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS joystick_teleop_node DESTINATION lib/${PROJECT_NAME})

ament_package()