#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class CompressedToRaw(Node):
    def __init__(self):
        super().__init__('compressed_to_raw')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        self.bridge = CvBridge()
        self.get_logger().info('Nodo iniciado: escuchando en /image_raw/compressed')

    def listener_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            image_msg.header = msg.header  # Copiamos el timestamp y frame_id
            self.publisher.publish(image_msg)
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CompressedToRaw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
