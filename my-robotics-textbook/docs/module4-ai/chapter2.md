---
sidebar_position: 2
title: "Computer Vision for Robotics"
---

import CodeSandbox from '@site/src/components/CodeSandbox';

# Computer Vision for Robotics

## Introduction to Robot Perception

Computer vision is a critical component of AI-driven robotics, enabling robots to perceive and understand their environment. In this chapter, we'll explore fundamental computer vision techniques for robotics applications.

## OpenCV with ROS 2

Here's an example of capturing and processing images from a camera in ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, 'camera/image_processed', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process the image (example: edge detection)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Convert back to ROS Image message
        processed_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
        processed_msg.header = msg.header

        # Publish processed image
        self.publisher.publish(processed_msg)

        # Display original and processed images
        cv2.imshow('Original', cv_image)
        cv2.imshow('Edges', edges)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()

    try:
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        image_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Object Detection with YOLO

For more advanced perception, robots can use deep learning models like YOLO for object detection:

```python
import cv2
import numpy as np

class YOLODetector:
    def __init__(self, config_path, weights_path, names_path):
        self.net = cv2.dnn.readNet(weights_path, config_path)
        with open(names_path, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    def detect_objects(self, image):
        height, width, channels = image.shape

        # Prepare image for detection
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)

        # Process outputs
        boxes = []
        confidences = []
        class_ids = []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply non-maximum suppression
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        return boxes, confidences, class_ids, indexes

# Example usage in a ROS 2 node
def process_with_detection(self, msg):
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    detector = YOLODetector('config.cfg', 'weights.weights', 'names.txt')

    boxes, confidences, class_ids, indexes = detector.detect_objects(cv_image)

    # Draw bounding boxes on image
    if len(indexes) > 0:
        for i in indexes.flatten():
            x, y, w, h = boxes[i]
            label = detector.classes[class_ids[i]]
            confidence = confidences[i]

            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image, f'{label} {confidence:.2f}',
                       (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Convert back to ROS message and publish
    result_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
    self.publisher.publish(result_msg)
```

<CodeSandbox
  title="ROS 2 Image Processing"
  language="python"
  code={`import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, 'camera/image_processed', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process the image (example: edge detection)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Convert back to ROS Image message
        processed_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
        processed_msg.header = msg.header

        # Publish processed image
        self.publisher.publish(processed_msg)

        # Display original and processed images
        cv2.imshow('Original', cv_image)
        cv2.imshow('Edges', edges)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()

    try:
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        image_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`}
/>

<CodeSandbox
  title="YOLO Object Detection"
  language="python"
  code={`import cv2
import numpy as np

class YOLODetector:
    def __init__(self, config_path, weights_path, names_path):
        self.net = cv2.dnn.readNet(weights_path, config_path)
        with open(names_path, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    def detect_objects(self, image):
        height, width, channels = image.shape

        # Prepare image for detection
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)

        # Process outputs
        boxes = []
        confidences = []
        class_ids = []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply non-maximum suppression
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        return boxes, confidences, class_ids, indexes`}
/>