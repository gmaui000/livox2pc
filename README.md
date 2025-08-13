# `livox2pc`

> Convert livox_ros_driver/CustomMsg point cloud messages from a rosbag into sensor_msgs/PointCloud2, saving results alongside original data into a new rosbag.

## Dependencies

* ROS Noetic
* [`livox_ros_driver`](https://github.com/Livox-SDK/livox_ros_driver) (in same workspace)
* [`Livox SDK`](https://github.com/Livox-SDK/Livox-SDK2.git) installed

### Usage

Edit `livox2pc.launch`：

```xml
<launch>
    <!-- 设置LiDAR话题 -->
    <arg name="livox_topic" default="/livox/lidar" />
    <!-- 设置为ROS包存放的路径 -->
    <arg name="rosbag_path" default="dataset/Retail_Street" />
    <node pkg="livox2pc" type="livox2pc_node" name="livox2pc_node" output="screen">
      <param name="livox_topic" type="string" value="$(arg livox_topic)" />
      <param name="rosbag_path" type="string" value="$(arg rosbag_path)" />
    </node>
</launch>
```

Then,

```bash
ros2 launch livox2pc livox2pc_launch.py rosbag_path:=dataset/Retail_Street
```
