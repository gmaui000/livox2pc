cmake_minimum_required(VERSION 3.8)
project(livox2pc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(${PROJECT_NAME} src/livox2pc.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  rosbag2_cpp
  livox_ros_driver
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
