---
sidebar_position: 1
title: "Introduction to Gazebo Simulation"
---

import CodeSandbox from '@site/src/components/CodeSandbox';

# Introduction to Gazebo Simulation

## What is Gazebo?

Gazebo is a powerful 3D simulation environment for robotics. It provides the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. Gazebo offers:

- A rich library of robots and environments
- High-fidelity physics simulation
- Multiple sensors with noise models
- Realistic rendering capabilities
- Integration with ROS and ROS 2

## Setting up a Basic Simulation

In this chapter, we'll learn how to create a simple robot model and simulate it in Gazebo.

```xml
<?xml version="1.0" ?>
<robot name="simple_robot">
  <link name="chassis">
    <visual>
      <geometry>
        <box size="1 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.3"/>
    </inertial>
  </link>
</robot>
```

This is a basic URDF (Unified Robot Description Format) model of a simple robot with a chassis.

<CodeSandbox
  title="Simple Robot URDF"
  code={`<?xml version="1.0" ?>
<robot name="simple_robot">
  <link name="chassis">
    <visual>
      <geometry>
        <box size="1 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.3"/>
    </inertial>
  </link>
</robot>`}
  language="xml"
/>