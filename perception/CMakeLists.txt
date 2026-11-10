cmake_minimum_required(VERSION 3.8)
project(image_converter_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_converter src/image_converter.cpp)

target_include_directories(image_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(image_converter
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
)

# Link OpenCV libraries
target_link_libraries(image_converter ${OpenCV_LIBRARIES})

install(TARGETS
  image_converter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()