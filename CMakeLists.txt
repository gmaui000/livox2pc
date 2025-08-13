cmake_minimum_required(VERSION 3.8)
project(livox2pc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Protobuf REQUIRED)

include(FindProtobuf)

# Proto 输出目录
set(PROTO_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${PROTO_OUT_DIR})

# 查找所有 proto 文件
file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/primitive/sensor/*.proto")

# 自动生成 C++ 源码
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
    ${PROTO_FILES}
    PROTOC_OUT_DIR ${PROTO_OUT_DIR}
    PROTOC_OPTIONS --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
)

# imu_converter
add_executable(imu_converter src/imu2livox.cpp ${PROTO_SRCS})
target_include_directories(imu_converter PRIVATE ${PROTO_OUT_DIR})
target_link_libraries(imu_converter ${Protobuf_LIBRARIES})
ament_target_dependencies(imu_converter
  rclcpp
  sensor_msgs
  livox_ros_driver
)
target_compile_options(imu_converter PRIVATE -Wall)

# livox2pc
add_executable(${PROJECT_NAME} src/livox2pc.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  rosbag2_cpp
  livox_ros_driver
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

install(TARGETS ${PROJECT_NAME} imu_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
