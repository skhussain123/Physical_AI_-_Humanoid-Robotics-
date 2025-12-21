---
sidebar_position: 100
title: "Documentation"
---

# Documentation for Physical AI & Humanoid Robotics Textbook

## Table of Contents
- [Getting Started](#getting-started)
- [System Architecture](#system-architecture)
- [Interactive Components](#interactive-components)
- [Code Execution](#code-execution)
- [Progress Tracking](#progress-tracking)
- [API Reference](#api-reference)

## Getting Started

The Physical AI & Humanoid Robotics textbook is built using Docusaurus 3.0+ with TypeScript support. It provides an interactive learning experience with embedded code execution environments for robotics frameworks.

### Prerequisites
- Node.js 18+ LTS
- npm 8+ (usually comes with Node.js)

### Installation
```bash
npm install
npm start
```

## System Architecture

The textbook follows a modular architecture with the following components:

- **Frontend**: Docusaurus-based static site
- **Interactive Components**: React components for code execution and simulation
- **Services**: Client-side services for progress tracking and code execution
- **Content**: Modular documentation in Markdown format

## Interactive Components

### CodeSandbox
The CodeSandbox component allows students to write and execute code directly in the textbook:

```jsx
<CodeSandbox
  title="ROS 2 Publisher Example"
  language="python"
  code={`import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    # ...`}
/>
```

### ProgressTracker
Tracks student progress across modules and chapters:

```jsx
<ProgressTracker modules={modules} />
```

### CertificateDisplay
Displays earned certificates:

```jsx
<CertificateDisplay studentName="Student" />
```

## Code Execution

The code execution environment supports multiple languages:
- Python (ROS 2, general robotics)
- JavaScript (ROSbridge, web clients)
- XML (URDF, configuration files)
- Bash (system commands)
- C++ (ROS nodes)

## Progress Tracking

The progress tracking system uses localStorage to persist student progress across sessions:

- Chapter completion status
- Module completion percentages
- Certificate achievements
- User preferences

## API Reference

### ProgressService
Client-side service for tracking and managing student progress.

#### Methods:
- `markChapterComplete(moduleId, chapterId)` - Mark a chapter as completed
- `isChapterComplete(moduleId, chapterId)` - Check if a chapter is completed
- `getModuleProgress(moduleId)` - Get progress for a module
- `isModuleComplete(moduleId)` - Check if a module is completed
- `getCertificates()` - Get all earned certificates
- `hasCertificate(moduleId)` - Check if a certificate has been earned

### CodeExecutionService
Client-side service for simulating code execution.

#### Methods:
- `executeCode(code, options)` - Execute code with specified options
- `validateCode(code, language)` - Validate code syntax