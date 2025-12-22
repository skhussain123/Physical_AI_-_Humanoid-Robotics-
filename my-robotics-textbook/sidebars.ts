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
        'module1/chapter1',
        'module1/chapter2',
        'module1/chapter3',
        'module1/chapter4',
        'module1/chapter5',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo Simulation',
      items: [
        'module2/chapter1',
        'module2/chapter2',
        'module2/chapter3',
        'module2/chapter4',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac',
      items: [
        'module3/chapter1',
        'module3/chapter2',
        'module3/chapter3',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: AI-Driven Robotics',
      items: [
        'module4/chapter1',
        'module4/chapter2',
        'module4/chapter3',
        'module4/chapter4',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Appendix',
      items: [
        'appendix/glossary',
        'appendix/references',
        'appendix/troubleshooting',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/autonomous-humanoid',
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;
