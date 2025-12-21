#!/usr/bin/env python3

"""
Simple Computer Vision Node for Robotics
This node processes camera images to detect objects and publish results
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class SimpleCVNode(Node):
    def __init__(self):
        super().__init__('simple_cv_node')

        # Create publisher for processed images
        self.image_publisher = self.create_publisher(Image, 'cv_node/image_processed', 10)

        # Create publisher for detection results
        self.result_publisher = self.create_publisher(String, 'cv_node/detection_results', 10)

        # Create subscription to camera images
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        self.get_logger().info('SimpleCVNode initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process image - detect circles as an example
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            gray = cv2.medianBlur(gray, 5)

            # Detect circles using HoughCircles
            circles = cv2.HoughCircles(
                gray,
                cv2.HOUGH_GRADIENT,
                dp=1,
                min_dist=50,
                param1=50,
                param2=30,
                min_radius=10,
                max_radius=100
            )

            detection_msg = "No objects detected"

            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                detection_msg = f"Detected {len(circles)} circles"

                # Draw circles on the image
                for (x, y, r) in circles:
                    cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(cv_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            else:
                detection_msg = "No circles detected"

            # Convert processed image back to ROS message
            processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            processed_msg.header = msg.header

            # Publish processed image
            self.image_publisher.publish(processed_msg)

            # Publish detection results
            result_msg = String()
            result_msg.data = detection_msg
            self.result_publisher.publish(result_msg)

            self.get_logger().info(f'Published: {detection_msg}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    simple_cv_node = SimpleCVNode()

    try:
        rclpy.spin(simple_cv_node)
    except KeyboardInterrupt:
        print('Shutting down computer vision node...')
    finally:
        simple_cv_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()