
# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from dataclasses import asdict
from digit360.interface.usb.usb import get_digit360_devices

def generate_launch_description():
    digit360_devices = get_digit360_devices()

    if not digit360_devices:
        raise RuntimeError("No DIGIT360 devices found!")

    device_info = digit360_devices[0] 
    device_parameters = asdict(device_info)

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=device_parameters['data']),
        DeclareLaunchArgument('device', default_value=device_parameters['ics']),
        DeclareLaunchArgument('audio_device', default_value=device_parameters['audio']),

        # Node for serial_io
        Node(
            package="d360",
            executable="serial_io",
            name="serial_io",
            parameters=[
                {"port": LaunchConfiguration('port')},
                {"reliable_publisher": False},
            ],
            output="screen",
            respawn=False,
            arguments=["--ros-args", "--log-level", "warn"],
        ),

        # Node for image publisher
        Node(
            package="d360",
            executable="image_pub_opencv",
            name="image_opencv_publisher_node",
            parameters=[
                {"device": LaunchConfiguration('device')}
            ],
            output="screen",
            respawn=False,
        ),

        # Node for audio publisher
        Node(
            package="d360",
            executable="audio_pub",
            name="audio_publisher_node",
            parameters=[
                {"device_name": LaunchConfiguration('audio_device')}
            ],
            output="screen",
            respawn=False,
        ),
    ])
