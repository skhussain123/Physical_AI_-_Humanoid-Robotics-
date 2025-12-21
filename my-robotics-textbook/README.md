# Physical AI & Humanoid Robotics Textbook

An AI-Native interactive textbook for learning Physical AI and Humanoid Robotics, featuring hands-on code execution environments for ROS 2, Gazebo, NVIDIA Isaac, and AI-driven robotics.

## Features

- Interactive code execution environments for robotics frameworks
- Progress tracking across 4 core modules
- Certificate generation upon module completion
- Responsive design with dark mode support
- WCAG 2.1 Level AA accessibility compliance
- Integration with robotics simulation tools

## Modules

1. **ROS 2 Fundamentals** - The robotic nervous system and communication framework
2. **Gazebo/Unity Simulation** - Physics-based simulation environments for robotics
3. **NVIDIA Isaac** - AI-powered robotics development platform
4. **AI-Driven Robotics** - Machine learning and AI techniques for robotics applications

## Getting Started

### Prerequisites

- Node.js 18+ LTS
- npm 8+ (usually comes with Node.js)

### Installation

```bash
npm install
```

### Local Development

```bash
npm start
```

This command starts a local development server and opens the application in your browser. Most changes are reflected live without having to restart the server.

### Build for Production

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static hosting service.

## Architecture

The textbook is built using Docusaurus 3.0+ with TypeScript support. Key components include:

- **Interactive Components**: React components for code execution and simulation
- **Services**: Client-side services for progress tracking and code execution
- **Content**: Modular documentation in Markdown format

## Contributing

This project follows a structured approach to feature development using specification-driven development principles. All contributions should maintain the high standards for code quality, accessibility, and user experience.
