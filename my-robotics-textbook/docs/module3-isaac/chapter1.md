---
sidebar_position: 1
title: "Introduction to NVIDIA Isaac"
---

import CodeSandbox from '@site/src/components/CodeSandbox';

# Introduction to NVIDIA Isaac

## What is NVIDIA Isaac?

NVIDIA Isaac is a comprehensive robotics platform that includes the Isaac SDK, Isaac Sim, and other tools for developing, simulating, and deploying AI-powered robots. Key components include:

- Isaac SDK: Software development kit for robotics applications
- Isaac Sim: High-fidelity simulation environment based on Omniverse
- Isaac ROS: Hardware acceleration and perception packages for ROS 2

## Getting Started with Isaac Sim

Isaac Sim provides a photorealistic simulation environment for robotics. Here's a basic Python script to control a robot in Isaac Sim:

```python
import omni
import carb
from pxr import Gf

# Example script to control a robot in Isaac Sim
class RobotController:
    def __init__(self):
        self.speed = 1.0
        print("Robot controller initialized")

    def move_forward(self, distance):
        print(f"Moving forward {distance} meters")
        # In a real Isaac Sim environment, this would control the robot
        return f"Moved forward {distance} meters"

    def turn(self, angle):
        print(f"Turning {angle} degrees")
        # In a real Isaac Sim environment, this would rotate the robot
        return f"Turned {angle} degrees"

# Example usage
controller = RobotController()
result1 = controller.move_forward(2.0)
result2 = controller.turn(90.0)
print(f"Results: {result1}, {result2}")
```

This script demonstrates the basic structure of controlling a robot in Isaac Sim.

<CodeSandbox
  title="Isaac Robot Controller"
  code={`import omni
import carb
from pxr import Gf

# Example script to control a robot in Isaac Sim
class RobotController:
    def __init__(self):
        self.speed = 1.0
        print("Robot controller initialized")

    def move_forward(self, distance):
        print(f"Moving forward {distance} meters")
        # In a real Isaac Sim environment, this would control the robot
        return f"Moved forward {distance} meters"

    def turn(self, angle):
        print(f"Turning {angle} degrees")
        # In a real Isaac Sim environment, this would rotate the robot
        return f"Turned {angle} degrees"

# Example usage
controller = RobotController()
result1 = controller.move_forward(2.0)
result2 = controller.turn(90.0)
print(f"Results: {result1}, {result2}")`}
/>