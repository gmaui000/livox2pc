cmake_minimum_required(VERSION 3.8)
project(livox2pc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
# Protobuf support
find_package(Protobuf REQUIRED)
if(Protobuf_FOUND)
  # Set output directory with proper structure
  set(PROTO_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/primitive/sensor)
  file(MAKE_DIRECTORY ${PROTO_OUT_DIR})

  # Generate protobuf sources for all proto files
  file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/primitive/sensor/*.proto")
  set(PROTO_SRCS)
  set(PROTO_HDRS)

  foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    set(PB_CC ${PROTO_OUT_DIR}/${PROTO_NAME}.pb.cc)
    set(PB_H ${PROTO_OUT_DIR}/${PROTO_NAME}.pb.h)
    
    add_custom_command(
      OUTPUT ${PB_CC} ${PB_H}
      COMMAND ${Protobuf_PROTOC_EXECUTABLE}
      ARGS --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/primitive/sensor
           --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
           --cpp_out=${PROTO_OUT_DIR}
           ${PROTO_FILE}
      DEPENDS ${PROTO_FILE}
      COMMENT "Generating ${PROTO_NAME} protobuf sources"
    )
    
    list(APPEND PROTO_SRCS ${PB_CC})
    list(APPEND PROTO_HDRS ${PB_H})
  endforeach()

  # Add generated sources to imu_converter target
  add_executable(imu_converter src/imu2livox.cpp ${PROTO_SRCS} ${PROTO_HDRS})
  target_link_libraries(imu_converter ${Protobuf_LIBRARIES})
  target_include_directories(imu_converter PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${PROTO_OUT_DIR}
  )
else()
  message(WARNING "protobuf not found - proto message support disabled")
  add_executable(imu_converter src/imu2livox.cpp)
endif()

add_executable(${PROJECT_NAME} src/livox2pc.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  rosbag2_cpp
  livox_ros_driver
)
ament_target_dependencies(imu_converter
  rclcpp
  sensor_msgs
  livox_ros_driver
)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)
target_compile_options(imu_converter PRIVATE -Wall)

install(TARGETS ${PROJECT_NAME} imu_converter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
