# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

#. Pressure AP.msg
#. Represents Pressure AP data from digit 360

std_msgs/Header header        		# Standard ROS header

uint64 ts 	                        # device time
uint32 seq


uint16[] ch_a
uint16[] ch_b
