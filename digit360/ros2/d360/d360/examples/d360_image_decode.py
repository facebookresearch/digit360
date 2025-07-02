import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
import av
import numpy as np
from numpy import int8
import threading


class ImageDecoder:
    def __init__(self, node, topic_name):
        self.node = node
        self.topic_name = topic_name
        self.lock = threading.Lock()
        self.bridge = CvBridge()

        try:
            self.codec = av.codec.CodecContext.create("h264", "r")
            self.node.get_logger().info(f"[{topic_name}] Decoder initialized.")
        except Exception as e:
            self.node.get_logger().error(
                f"[{topic_name}] Failed to create decoder: {e}"
            )
            raise

        # Set up publisher for decoded image
        self.publisher = self.node.create_publisher(
            Image, topic_name.replace("image_encode", "image_compressed"), 10
        )

        # Subscribe to compressed bitstream
        self.subscription = self.node.create_subscription(
            UInt8MultiArray, topic_name, self.callback, 10
        )

    def callback(self, msg):
        packets = None
        with self.lock:
            try:
                packets = self.codec.parse(bytes(np.array(msg.data, dtype=np.int8)))
            except Exception as e:
                self.node.get_logger().warn(f"[{self.topic_name}] Parse error: {e}")
                return
        for packet in packets:
            try:
                frames = self.codec.decode(packet)
                for frame in frames:
                    rgb = frame.to_rgb().to_ndarray()
                    ros_img = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
                    ros_img.header.stamp = self.node.get_clock().now().to_msg()
                    self.publisher.publish(ros_img)
            except Exception as e:
                self.node.get_logger().warn(f"[{self.topic_name}] Decode error: {e}")


class MultiH264Bridge(Node):
    def __init__(self):
        super().__init__("multi_h264_bridge")
        self.decoders = {}

        self.declare_parameter('num_d360', 8)
        sensor_count = self.get_parameter('num_d360').get_parameter_value().integer_value

        # Initialize decoders for all possible topics
        for i in range(sensor_count):
            topic = f"d360_{i}/image_encode"
            try:
                decoder = ImageDecoder(self, topic)
                self.decoders[topic] = decoder
            except Exception as e:
                self.get_logger().warn(f"Could not initialize decoder for {topic}: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = MultiH264Bridge()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()