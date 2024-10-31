# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

#. GasHTData.msg
#. Represents Gas and Humidity/Temperature data

std_msgs/Header header  # Standard ROS header

uint64 ts 	            # device time

uint64 sensor_ts        # sensor time

float32 temperature
float32 pressure
float32 humidity
float32 gas
uint8 gas_index

