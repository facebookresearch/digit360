# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
cv2.setNumThreads(0)

class CameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__('d360_image_node')
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, 'image_raw', 1)
        self.publisher_compressed = self.create_publisher(CompressedImage, 'image_raw/compressed', 1)
        device = self.declare_parameter('device', '/dev/video0').get_parameter_value().string_value
        self.cap = cv2.VideoCapture(device)
        ret, frame = self.cap.read()
        assert ret, f"Failed to open DIGIT360 camera image capture system *{device}*"

        self.thread = threading.Thread(target=self.publish_camera_image)
        self.thread.start()

    def publish_camera_image(self) -> None:
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                if self.publisher.get_subscription_count():
                    image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
                    image_msg.header.stamp = self.get_clock().now().to_msg()
                    self.publisher.publish(image_msg)
                if self.publisher_compressed.get_subscription_count():
                    cimg_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpeg")
                    cimg_msg.header.stamp = self.get_clock().now().to_msg()
                    self.publisher_compressed.publish(cimg_msg)
            else:
                self.get_logger().error("Failed to read frame from DIGIT360 camera image capture system", throttle_duration_sec=10)
                break
                
        self.thread.join()

def main(args: list=None) -> None:
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
