# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
import os
from ament_index_python.packages import get_package_share_directory

foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name = f"foxglove",
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
))
    
def generate_launch_description() -> LaunchDescription:

    ld = LaunchDescription([
        foxglove,
        d360_min,
    ])
    return ld



if __name__ == '__main__':
    print(generate_launch_description())