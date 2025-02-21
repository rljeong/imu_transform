import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            "yaw_offset_",
            default_value="1.57079632679",
            description="IMU Yaw rotation offset in radians"
        ),

        Node(
            package="imu_transform",
            executable="imu_transform",
            name="imu_transform",
            output="screen",
            parameters=[{"yaw_offset_": LaunchConfiguration("yaw_offset_")}],
        ),
    ])