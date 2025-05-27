# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
import cv2
import numpy as np
import PyNvVideoCodec as nvc
import pycuda.driver as cuda
import pycuda.autoinit

class CompressedCameraPublisher(Node):
    def __init__(self):
        super().__init__('d360_compressed_image_node')
        self.publisher = self.create_publisher(UInt8MultiArray, 'h264_bitstream', 10)

        self.declare_parameter('device', '/dev/video0')
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.get_logger().info(f"Using video device: {self.device}")

        self.WIDTH = 1280
        self.HEIGHT = 720

        # Initialize CUDA
        try:
            cuda.init()
            device_cuda = cuda.Device(0)
            self.ctx = device_cuda.retain_primary_context()
            self.ctx.push()
            self.stream = cuda.Stream()
            self.get_logger().info("CUDA initialized.")
        except Exception as e:
            self.get_logger().error(f"CUDA initialization failed: {e}")
            raise

        # Initialize encoder
        try:
            config_params = {"idrperiod": 30, "preset": "P4", "gop": 30, "repeatspspps": 1}
            self.encoder = nvc.CreateEncoder(self.WIDTH, self.HEIGHT, "NV12", True, **config_params)
            self.get_logger().info("H.264 encoder initialized.")
        except Exception as e:
            self.get_logger().error(f"Encoder creation failed: {e}")
            raise

        # Open specified camera
        self.cap = cv2.VideoCapture(self.device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera at {self.device}")
            raise RuntimeError(f"Camera not accessible at {self.device}")

        self.get_logger().info("Camera initialized.")
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        assert ret, f"Failed to open DIGIT360 camera image capture system *{self.device}*"
        if not ret:
            self.get_logger().warn("Failed to read frame from camera.")
            return

        try:
            yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
            y = yuv[:, :, 0]
            u = yuv[::2, ::2, 1]
            v = yuv[::2, ::2, 2]

            y_flat = y.flatten()
            uv = np.empty((u.size + v.size,), dtype=np.uint8)
            uv[0::2] = u.flatten()
            uv[1::2] = v.flatten()
            nv12 = np.concatenate((y_flat, uv))

            bitstream = self.encoder.Encode(nv12)

            if bitstream:
                msg = UInt8MultiArray()
                msg.data = list(bitstream)
                self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Encoding or publishing failed: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down and releasing camera.")
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CompressedCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
