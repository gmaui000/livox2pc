from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox2pc',
            executable='imu_converter',
            name='imu2livox',
            output='screen',
            parameters=[{
                # Add any parameters here if needed
            }],
            remappings=[
                # Add any topic remappings here if needed
            ]
        )
    ])
