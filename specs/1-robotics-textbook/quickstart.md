# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Prerequisites

Before getting started with the development of the interactive robotics textbook, ensure you have the following installed:

- **Node.js**: Version 18.x or higher (LTS recommended)
- **npm**: Version 8.x or higher (usually comes with Node.js)
- **Git**: For version control
- **A modern code editor** (VS Code recommended)

## Setup Instructions

### 1. Clone or Initialize the Repository

If starting fresh:
```bash
npx create-docusaurus@latest my-robotics-textbook classic --typescript
cd my-robotics-textbook
```

If working with existing repository:
```bash
git clone <repository-url>
cd my-robotics-textbook
npm install
```

### 2. Install Additional Dependencies

```bash
npm install --save-dev @docusaurus/types typescript
npm install
```

### 3. Verify Installation

```bash
npm run build
npm run start
```

Your Docusaurus site should now be running on `http://localhost:3000`.

## Project Structure Overview

```
my-robotics-textbook/
├── docs/                 # Content for textbook modules
│   ├── module1-ros2/     # ROS 2 module content
│   ├── module2-gazebo/   # Gazebo/Unity module content
│   ├── module3-isaac/    # NVIDIA Isaac module content
│   └── module4-ai/       # AI-Driven Robotics module content
├── src/
│   ├── components/       # Interactive components (CodeSandbox, etc.)
│   ├── pages/            # Custom pages (dashboard, login, etc.)
│   └── theme/            # Custom theme components
├── static/               # Static assets
├── docusaurus.config.ts  # Main configuration
├── package.json          # Project dependencies
└── tsconfig.json         # TypeScript configuration
```

## Adding Your First Module

1. Create a new directory in the `docs/` folder:
```bash
mkdir docs/module1-ros2
```

2. Add your first chapter:
```bash
echo "# Introduction to ROS 2

This is the beginning of our ROS 2 module..." > docs/module1-ros2/chapter1.md
```

3. Update `docusaurus.config.ts` to include your new module in the sidebar.

## Key Configuration Files

### docusaurus.config.ts
This file contains all the main configuration for your Docusaurus site, including:
- Site metadata (title, description, favicon)
- Theme configuration
- Plugin configuration
- Sidebar navigation

### tsconfig.json
TypeScript configuration for your project, including:
- Compiler options
- Path aliases
- Type checking settings

## Running the Development Server

```bash
npm run start
```

This starts a local development server and opens your site in a browser. Most changes are reflected live without having to restart the server.

## Building for Production

```bash
npm run build
```

This command generates static content into the `build` directory and can be served using any static hosting service.

## Deploying to GitHub Pages

1. Ensure your GitHub repository is set up
2. Update the `deployment` section in `docusaurus.config.ts`:
   - `organizationName`: Your GitHub username or organization
   - `projectName`: Your repository name
3. Run the deployment command:
```bash
npm run deploy
```

## Interactive Components

For the code sandbox and other interactive elements:

1. Create components in `src/components/`
2. Import and use them in your markdown files:

```md
import CodeSandbox from '@site/src/components/CodeSandbox';

<CodeSandbox code={`console.log('Hello ROS 2!');`} />
```

## Next Steps

1. Begin adding content to your modules in the `docs/` directory
2. Create custom components for interactive learning experiences
3. Implement user progress tracking
4. Add accessibility features
5. Test across different browsers and platforms