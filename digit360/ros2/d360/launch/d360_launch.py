# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

image_pub_type_arg = DeclareLaunchArgument(
    'image_pub_type',
    default_value='encoded',
    description='Select image publisher: raw or encoded'
)

foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name = "foxglove",
        parameters=[
            {"port": 8766}
        ],
        arguments=["--ros-args", "--log-level", "warn"]
    )

d360_min = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("d360"),
            "launch/d360_min_launch.py",
        )
    ),
    launch_arguments={'image_pub_type': LaunchConfiguration('image_pub_type')}.items()
)
    
def generate_launch_description() -> LaunchDescription:

    return LaunchDescription([
        image_pub_type_arg,  # make sure we declare it here
        foxglove,
        d360_min,
    ])

if __name__ == '__main__':
    print(generate_launch_description())