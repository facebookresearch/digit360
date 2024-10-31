# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.


#. ImuEulerD360.msg
#. Represents euler fused 9dof imu data from digit 360

std_msgs/Header header      # Standard ROS header

uint64 ts 	            # device time

uint64 sensor_ts            # sensor time

float32 heading
float32 pitch
float32 roll



