#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "primitive/sensor/sensor_msgs.pb.h"

using namespace std::chrono_literals;

class ImuConverter : public rclcpp::Node {
public:
  ImuConverter() : Node("imu_converter") {
    // Create subscriber for raw protobuf data
    proto_imu_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "/livox/imu", 10,
      [this](const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
        primitive::sensor::Imu proto_msg;
        if (proto_msg.ParseFromArray(msg->data.data(), msg->data.size())) {
          auto std_imu = convertToStandardImu(proto_msg);
          std_imu_pub_->publish(std_imu);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse protobuf IMU message");
        }
      });
      
    std_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
  }

private:
  bool parseProtoFromROS(const sensor_msgs::msg::Imu& ros_msg, primitive::sensor::Imu& proto_msg) {
    // Parse ROS message to proto (if needed)
    // This would be custom parsing logic
    return true;
  }

  sensor_msgs::msg::Imu convertToStandardImu(const primitive::sensor::Imu& proto_msg) {
    auto std_imu = sensor_msgs::msg::Imu();
    
    // Convert header
    std_imu.header.stamp = this->now();
    std_imu.header.frame_id = "imu_link";
    
    // Convert orientation
    std_imu.orientation.x = proto_msg.orientation().x();
    std_imu.orientation.y = proto_msg.orientation().y();
    std_imu.orientation.z = proto_msg.orientation().z();
    std_imu.orientation.w = proto_msg.orientation().w();
    
    // Convert angular velocity
    std_imu.angular_velocity.x = proto_msg.angular().x();
    std_imu.angular_velocity.y = proto_msg.angular().y();
    std_imu.angular_velocity.z = proto_msg.angular().z();
    
    // Convert linear acceleration
    std_imu.linear_acceleration.x = proto_msg.acceleration().x();
    std_imu.linear_acceleration.y = proto_msg.acceleration().y();
    std_imu.linear_acceleration.z = proto_msg.acceleration().z();
    
    // Convert covariance matrices
    if (proto_msg.orientation_covariance_size() >= 9) {
      for (int i = 0; i < 9; i++) {
        std_imu.orientation_covariance[i] = proto_msg.orientation_covariance(i);
      }
    }
    
    if (proto_msg.angular_covariance_size() >= 9) {
      for (int i = 0; i < 9; i++) {
        std_imu.angular_velocity_covariance[i] = proto_msg.angular_covariance(i);
      }
    }
    
    if (proto_msg.acceleration_covariance_size() >= 9) {
      for (int i = 0; i < 9; i++) {
        std_imu.linear_acceleration_covariance[i] = proto_msg.acceleration_covariance(i);
      }
    }

    return std_imu;
  }

  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr proto_imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr std_imu_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuConverter>());
  rclcpp::shutdown();
  return 0;
}
