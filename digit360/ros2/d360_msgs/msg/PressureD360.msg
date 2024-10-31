# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

#. Pressure.msg
#. Represents Pressure data from digit 360

std_msgs/Header header        		# Standard ROS header

uint64 ts 	                        # device time

float64 pressure
float64 temperature
