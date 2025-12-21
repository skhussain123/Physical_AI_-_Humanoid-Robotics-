import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar for the robotics textbook
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module1-ros2/chapter1',
        'module1-ros2/chapter2',
        'module1-ros2/chapter3',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo Simulation',
      items: [
        'module2-gazebo/chapter1',
        'module2-gazebo/chapter2',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      items: [
        'module3-isaac/chapter1',
        'module3-isaac/chapter2',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: AI-Driven Robotics',
      items: [
        'module4-ai/chapter1',
        'module4-ai/chapter2',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Resources',
      items: [
        'documentation',
        'accessibility',
      ],
      collapsed: true,
    },
  ],
};

export default sidebars;
