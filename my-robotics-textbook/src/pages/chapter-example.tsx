import React from 'react';
import Layout from '@theme/Layout';
import Chapter from '@site/src/components/Chapter/Chapter';
import CodeSandbox from '@site/src/components/CodeSandbox/CodeSandbox';
import CodeExecutor from '@site/src/components/CodeSandbox/CodeExecutor';

export default function ChapterWithCodeExample() {
  const chapterContent = `
    <h2>Interactive Learning Experience</h2>
    <p>This chapter demonstrates how code examples are integrated directly into the textbook content.</p>
    <p>You can run the code examples below directly in your browser without needing to set up any local environment.</p>
  `;

  return (
    <Layout title="Chapter with Interactive Code">
      <Chapter
        title="Interactive Code Example"
        moduleTitle="ROS 2 Fundamentals"
        chapterNumber={1}
        content={chapterContent}
      >
        <div style={{ marginTop: '2rem' }}>
          <h3>Try ROS 2 Publisher Example</h3>
          <CodeSandbox
            title="ROS 2 Publisher Node"
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
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1`}
          />
        </div>

        <div style={{ marginTop: '2rem' }}>
          <h3>Another Code Example</h3>
          <CodeExecutor
            code={`# Python example
def fibonacci(n):
    if n <= 1:
        return n
    else:
        return fibonacci(n-1) + fibonacci(n-2)

# Calculate first 10 fibonacci numbers
for i in range(10):
    print(f"F({i}) = {fibonacci(i)}")`}
            language="python"
          />
        </div>
      </Chapter>
    </Layout>
  );
}