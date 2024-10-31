# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

#. ImuQuatD360.msg
#. Represents quaternion fused 9dof imu data from digit 360

std_msgs/Header header      # Standard ROS header

uint64 ts 	                # device time

uint64 sensor_ts            # sensor time

geometry_msgs/Quaternion quat
float32 accuracy



