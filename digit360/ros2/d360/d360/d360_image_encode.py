#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from foxglove_msgs.msg import CompressedVideo
import cv2
import numpy as np
import PyNvVideoCodec as nvc 

class CompressedCameraPublisher(Node):
    def __init__(self):
        super().__init__('d360_compressed_gpu')
        self.publisher = self.create_publisher(CompressedVideo, 'encoded_h264', 10)
        self.declare_parameter('device', '/dev/video0')
        device = self.get_parameter('device').get_parameter_value().string_value
        self.get_logger().info(f"Using Digit 360 video device: {device}")
        self.WIDTH = 1280
        self.HEIGHT = 720

        try:
            # NVENC Encoding Parameters
            config_params = {
                "codec": "h264",
                "preset": "P1",
                "idrperiod": 1, 
                "repeatspspps": 1,
                "tuning_info": "low_latency",
                "rc": "cbr",
                "multipass": "fullres",
                "fps": 30,
                "bf": 1,
                # "gop": 15,
                "bitrate": "4M",
                "maxbitrate": "5M",
                "vbvinit": "2M",
                "vbvbufsize": "2M",
                "initqp": 32,
                "qmin": "0,0,0",
                "qmax": "0,0,0",
                "initqp": "0,0,0",
                }
            
            self.encoder = nvc.CreateEncoder(self.WIDTH, self.HEIGHT, "NV12", True, **config_params)
            self.get_logger().info("H.264 encoder initialized.")
        except Exception as e:
            self.get_logger().error(f"Encoder creation failed: {e}")
            raise

        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open Digit 360 at {device}")
            raise RuntimeError(f"Digit 360 not accessible at {device}, could not open device.")

        self.get_logger().info(f"Digit 360 {device} - ICS initialized.")
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read video frame from Digit 360.")
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
                msg = CompressedVideo()
                msg.timestamp = self.get_clock().now().to_msg()
                msg.frame_id = "camera_frame"  
                msg.data = bytearray(bitstream)  
                msg.format = "h264"  
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