---
sidebar_position: 1
title: "Introduction to ROS 2"
---

import CodeSandbox from '@site/src/components/CodeSandbox';

# Introduction to ROS 2

## What is ROS 2?

Robot Operating System 2 (ROS 2) is the next generation of the Robot Operating System, designed to be more robust, scalable, and suitable for production environments. Unlike its predecessor, ROS 2 is built on DDS (Data Distribution Service) for communication, providing better real-time performance and support for distributed systems.

## Key Concepts

### Nodes
Nodes are the fundamental building blocks of ROS 2. They are processes that perform computation and communicate with other nodes through messages. In ROS 2, nodes are more robust and can be distributed across multiple machines.

### Topics
Topics are named buses over which nodes exchange messages. They implement a publish-subscribe communication pattern, where publishers send messages to a topic and subscribers receive messages from a topic.

### Services
Services provide a request-response communication pattern. A client sends a request to a service, which processes the request and returns a response.

### Actions
Actions are used for long-running tasks that require feedback and the ability to cancel the task before completion.

## Getting Started with ROS 2

Let's create our first ROS 2 node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This example demonstrates a basic publisher node that publishes a "Hello World" message every 0.5 seconds.

## Exercise

Try modifying the code above to publish a different message every time, such as counting in a sequence or publishing a timestamp.

<CodeSandbox
  title="ROS 2 Publisher Example"
  code={`import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1`}
/>