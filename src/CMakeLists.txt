cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_multithreaded_executor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(multithreaded_executor multithreaded_executor.cpp pub.cpp sub.cpp)
ament_target_dependencies(multithreaded_executor rclcpp std_msgs sensor_msgs
	pcl_conversions)

target_link_libraries( multithreaded_executor
    ${PCL_LIBRARIES}
)

install(TARGETS
  multithreaded_executor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
