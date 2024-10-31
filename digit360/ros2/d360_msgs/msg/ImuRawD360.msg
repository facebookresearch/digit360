# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

#. ImuRawD360.msg
#. Represents raw imu data from digit 360

std_msgs/Header header  # Standard ROS header

uint64 ts 	            # device time

uint64 sensor_ts        # sensor time

uint8 type         		# Type of data ( 1:ACC , 2:GYRO , 3:MAG, 6:LINACC)

float32 x
float32 y
float32 z



