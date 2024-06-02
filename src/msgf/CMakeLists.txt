cmake_minimum_required(VERSION 3.5)
project(message_filters_sample_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(message_filters_sample main.cpp)

ament_target_dependencies(message_filters_sample
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  OpenCV
)

install(TARGETS
  message_filters_sample
  DESTINATION lib/${PROJECT_NAME})

ament_package()


