# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import serial

import rclpy
from threading import Thread
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from d360_msgs.msg import GasHTData

class GasReader(Node, Thread):
    def __init__(self):
        Node.__init__(self, "gas")
        Thread.__init__(self)
        self.declare_parameter("port", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING, read_only=True))
        self.dev = serial.Serial(self.get_parameter("port").value, baudrate=115200)
        self.pub = self.create_publisher(GasHTData, "gas", 1)
        self.start()

    def run(self):
        while rclpy.ok():
            try:
                raw = self.dev.readline()
                raw = raw.decode().strip()
                line = raw.split(',')
                msg = GasHTData()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.sensor_ts = int(line[0])
                msg.temperature = float(line[1])
                msg.pressure = float(line[2])
                msg.humidity = float(line[3])
                msg.gas = float(line[4])
                msg.gas_index = int(line[5])
                self.pub.publish(msg)
            except (IndexError, ValueError) as e:
                print("decoding error, raw data from line was - ", raw)
                print(e)


def main():
    rclpy.init()
    node = GasReader()
    rclpy.spin(node)

if __name__ == '__main__':
    main()