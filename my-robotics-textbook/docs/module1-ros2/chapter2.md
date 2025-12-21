---
sidebar_position: 2
title: "ROS 2 Publishers and Subscribers"
---

import CodeSandbox from '@site/src/components/CodeSandbox';

# ROS 2 Publishers and Subscribers

## Understanding Publish-Subscribe Pattern

The publish-subscribe pattern is a messaging pattern where publishers send messages to a topic without knowledge of which subscribers, if any, there are. Subscribers express interest in one or more topics and only receive messages that are of interest without knowledge of which publishers, if any, there are.

## Creating a Publisher Node

Let's create a publisher node that publishes sensor data:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(String, 'sensor_data', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Sensor publisher node initialized')

    def timer_callback(self):
        # Simulate sensor data
        sensor_value = random.uniform(0.0, 100.0)
        msg = String()
        msg.data = f'Sensor Reading: {sensor_value:.2f}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Subscriber Node

Now let's create a subscriber node that receives and processes the sensor data:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Sensor subscriber node initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received sensor data: "{msg.data}"')
        # Process the sensor data here
        processed_data = self.process_sensor_data(msg.data)
        self.get_logger().info(f'Processed data: "{processed_data}"')

    def process_sensor_data(self, data):
        # Simple processing: extract value and categorize
        try:
            value = float(data.split(': ')[1])
            if value < 30:
                return f'LOW: {value:.2f}'
            elif value < 70:
                return f'MEDIUM: {value:.2f}'
            else:
                return f'HIGH: {value:.2f}'
        except:
            return f'INVALID: {data}'

def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()

    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running Publisher and Subscriber

To run these nodes:

1. Open two terminal windows
2. In the first terminal, run the publisher: `ros2 run your_package sensor_publisher`
3. In the second terminal, run the subscriber: `ros2 run your_package sensor_subscriber`

You should see the publisher sending data and the subscriber receiving and processing it.

## Exercise

Modify the subscriber to maintain a running average of the sensor values and publish an alert when the value goes above a certain threshold.

<CodeSandbox
  title="ROS 2 Publisher Example"
  code={`import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(String, 'sensor_data', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Sensor publisher node initialized')

    def timer_callback(self):
        # Simulate sensor data
        sensor_value = random.uniform(0.0, 100.0)
        msg = String()
        msg.data = f'Sensor Reading: {sensor_value:.2f}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`}
/>

<CodeSandbox
  title="ROS 2 Subscriber Example"
  code={`import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Sensor subscriber node initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received sensor data: "{msg.data}"')
        # Process the sensor data here
        processed_data = self.process_sensor_data(msg.data)
        self.get_logger().info(f'Processed data: "{processed_data}"')

    def process_sensor_data(self, data):
        # Simple processing: extract value and categorize
        try:
            value = float(data.split(': ')[1])
            if value < 30:
                return f'LOW: {value:.2f}'
            elif value < 70:
                return f'MEDIUM: {value:.2f}'
            else:
                return f'HIGH: {value:.2f}'
        except:
            return f'INVALID: {data}'

def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()

    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()`}
/>