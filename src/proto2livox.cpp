#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "primitive/sensor/sensor_msgs.pb.h"
#include "livox_ros_driver/msg/custom_msg.hpp"
#include "foxglove/PointCloud.pb.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <variant>

using namespace std::chrono_literals;

class Proto2LivoxConverter : public rclcpp::Node {
public:
  Proto2LivoxConverter() : Node("proto2livox_converter") {
    // Declare output format parameter
    this->declare_parameter<std::string>("output_format", "livox");
    output_format_ = this->get_parameter("output_format").as_string();
    
    if (output_format_ != "livox" && output_format_ != "std") {
      RCLCPP_ERROR(this->get_logger(), "Invalid output_format: %s, must be 'livox' or 'std'", 
                  output_format_.c_str());
      throw std::runtime_error("Invalid output_format parameter");
    }
    RCLCPP_INFO(this->get_logger(), "Output format set to: %s (valid options: livox, std)", 
               output_format_.c_str());
    // Create subscriber for raw protobuf data
    proto_imu_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "/imu", 10,
      [this](const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
        asp_sensor::Imu proto_msg;
        if (proto_msg.ParseFromArray(msg->data.data(), msg->data.size())) {
          auto livox_imu = convertToLivoxImu(proto_msg);
          livox_imu_pub_->publish(livox_imu);
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse protobuf IMU message");
        }
      });
      
    // Initialize QoS profile
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability_volatile();

    // Create publishers
    livox_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu", qos);
    
    if (output_format_ == "livox") {
      livox_pub_ = this->create_publisher<livox_ros_driver::msg::CustomMsg>("/livox/lidar", qos);
    } else {
      std_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/pointcloud2", qos);
    }
    
    // Create LiDAR subscriber
    proto_lidar_sub_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      "/main/ruby/lidar_points", qos,
      [this](const std_msgs::msg::ByteMultiArray::SharedPtr msg) {
        foxglove::PointCloud proto_msg;
        if (proto_msg.ParseFromArray(msg->data.data(), msg->data.size())) {
          if (output_format_ == "livox") {
            auto livox_lidar = convertToLivoxLidar(proto_msg);
            livox_pub_->publish(livox_lidar);
          } else {
            auto std_lidar = convertToStandardLidar(proto_msg);
            std_pub_->publish(std_lidar);
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse protobuf IMU message");
        }
      });
  }

private:
  sensor_msgs::msg::Imu convertToLivoxImu(const asp_sensor::Imu& proto_msg) {
    auto msg = sensor_msgs::msg::Imu();
    
    // Convert header
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";
    
    // Convert orientation
    msg.orientation.x = proto_msg.orientation().x();
    msg.orientation.y = proto_msg.orientation().y();
    msg.orientation.z = proto_msg.orientation().z();
    msg.orientation.w = proto_msg.orientation().w();
    
    // Convert angular velocity
    msg.angular_velocity.x = proto_msg.angular().x();
    msg.angular_velocity.y = proto_msg.angular().y();
    msg.angular_velocity.z = proto_msg.angular().z();
    
    // Convert linear acceleration
    msg.linear_acceleration.x = proto_msg.acceleration().x();
    msg.linear_acceleration.y = proto_msg.acceleration().y();
    msg.linear_acceleration.z = proto_msg.acceleration().z();
    
    // Convert covariance matrices
    if (proto_msg.orientation_covariance_size() >= 9) {
      for (int i = 0; i < 9; i++) {
        msg.orientation_covariance[i] = proto_msg.orientation_covariance(i);
      }
    }
    
    if (proto_msg.angular_covariance_size() >= 9) {
      for (int i = 0; i < 9; i++) {
        msg.angular_velocity_covariance[i] = proto_msg.angular_covariance(i);
      }
    }
    
    if (proto_msg.acceleration_covariance_size() >= 9) {
      for (int i = 0; i < 9; i++) {
        msg.linear_acceleration_covariance[i] = proto_msg.acceleration_covariance(i);
      }
    }

    return msg;
  }

  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr proto_imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr livox_imu_pub_;

  livox_ros_driver::msg::CustomMsg convertToLivoxLidar(const foxglove::PointCloud& proto_msg) {
    auto msg = livox_ros_driver::msg::CustomMsg();
    msg.header.stamp = rclcpp::Time(proto_msg.timestamp().seconds(), proto_msg.timestamp().nanos());
    msg.header.frame_id = proto_msg.frame_id();
    
    // Parse point cloud data
    const auto& fields = proto_msg.fields();
    const auto& data = proto_msg.data();
    size_t point_count = data.size() / proto_msg.point_stride();
    msg.points.resize(point_count);

    // Find field offsets
    int x_offset = -1, y_offset = -1, z_offset = -1, intensity_offset = -1;
    for (const auto& field : fields) {
      if (field.name() == "x") x_offset = field.offset();
      else if (field.name() == "y") y_offset = field.offset();
      else if (field.name() == "z") z_offset = field.offset();
      else if (field.name() == "intensity") intensity_offset = field.offset();
    }

    if (x_offset == -1 || y_offset == -1 || z_offset == -1) {
      RCLCPP_ERROR(this->get_logger(), "Missing required fields (x,y,z) in point cloud");
      return msg;
    }

    // Convert each point
    for (size_t i = 0; i < point_count; ++i) {
      const uint8_t* point_data = reinterpret_cast<const uint8_t*>(data.data()) + i * proto_msg.point_stride();
      auto& point = msg.points[i];

      point.x = *reinterpret_cast<const float*>(point_data + x_offset);
      point.y = *reinterpret_cast<const float*>(point_data + y_offset);
      point.z = *reinterpret_cast<const float*>(point_data + z_offset);

      if (intensity_offset != -1) {
        point.reflectivity = *reinterpret_cast<const float*>(point_data + intensity_offset);
      } else {
        point.reflectivity = 0.0f;
      }

      point.offset_time = 0;  // TODO: Calculate proper offset time if available
      point.tag = 0;
      point.line = 0;
    }

    // Apply pose transformation if needed
    if (proto_msg.has_pose()) {
      // TODO: Implement pose transformation if needed
      RCLCPP_WARN(this->get_logger(), "Pose transformation not yet implemented");
    }

    return msg;
  }

  sensor_msgs::msg::PointCloud2 convertToStandardLidar(const foxglove::PointCloud& proto_msg) {
    auto msg = sensor_msgs::msg::PointCloud2();
    msg.header.stamp = rclcpp::Time(proto_msg.timestamp().seconds(), proto_msg.timestamp().nanos());
    msg.header.frame_id = proto_msg.frame_id();
    msg.height = 1;
    msg.width = proto_msg.data().size() / proto_msg.point_stride();

    // Set fields
    msg.fields.resize(7);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.fields[2].name = "ring";
    msg.fields[2].offset = 12;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::UINT16;
    msg.fields[2].count = 1;

    msg.fields[2].name = "column";
    msg.fields[2].offset = 14;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::UINT16;
    msg.fields[2].count = 1;

    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 16;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
    msg.fields[3].count = 1;

    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 17;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
    msg.fields[3].count = 1;

    msg.point_step = 18;
    msg.row_step = msg.width * msg.point_step;
    const auto& proto_data = proto_msg.data();
    msg.data.assign(proto_data.begin(), proto_data.end());

    return msg;
  }

  std::string output_format_;
  rclcpp::Publisher<livox_ros_driver::msg::CustomMsg>::SharedPtr livox_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr std_pub_;
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr proto_lidar_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Proto2LivoxConverter>());
  rclcpp::shutdown();
  return 0;
}
