cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(perception_node src/perception_node.cpp)

ament_target_dependencies(
  perception_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf2_ros
)

target_link_libraries(
  perception_node
  ${OpenCV_LIBS}
)

install(TARGETS
  perception_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()