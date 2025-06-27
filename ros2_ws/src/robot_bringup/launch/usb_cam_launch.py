#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Image width'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Image height'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Camera framerate'
    )
    
    # USB camera node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'framerate': LaunchConfiguration('framerate'),
            'pixel_format': 'yuyv',
            'camera_frame_id': 'usb_cam',
            'io_method': 'mmap'
        }],
        output='screen'
    )
    
    return LaunchDescription([
        video_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        usb_cam_node
    ])