# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

**Feature**: Physical AI & Humanoid Robotics Textbook
**Branch**: `1-physical-ai-textbook`
**Created**: 2025-12-06
**Status**: Complete
**Author**: Claude Code

---

## Overview

This quickstart guide provides essential information for setting up, developing, and deploying the Physical AI & Humanoid Robotics Textbook. The textbook is built using Docusaurus with React and TypeScript components for interactive content.

## Prerequisites

### System Requirements
- **Operating System**: macOS, Windows, or Linux
- **Node.js**: Version 18.x or higher (LTS recommended)
- **npm**: Version 8.x or higher (typically bundled with Node.js)
- **Git**: Version 2.0 or higher
- **Python**: Version 3.8 or higher (for some build dependencies)

### Hardware Requirements
- **Minimum**: 8GB RAM, 25GB free disk space
- **Recommended**: 16GB RAM, 50GB free disk space, dedicated GPU for 3D visualization development
- **OS Compatibility**: All major operating systems (Windows, macOS, Linux)

## Development Environment Setup

### 1. Clone the Repository

```bash
# Clone the repository to your local machine
git clone <repository-url>
cd ai_native_book

# If you're continuing development on an existing branch
git checkout 1-physical-ai-textbook
```

### 2. Install Dependencies

```bash
# Install all project dependencies
npm install

# Install additional packages for robotics visualizations
npm install three @types/three d3 @types/d3 chart.js react-chartjs-2

# Install dev dependencies
npm install --save-dev @testing-library/react @testing-library/jest-dom \
  eslint prettier eslint-config-prettier jest
```

### 3. Verify Installation

```bash
# Check Node.js version
node --version

# Check npm version
npm --version

# Verify all dependencies are installed
npm ls --depth=0
```

## Running the Development Server

### 1. Start Development Server

```bash
# Start the Docusaurus development server
npm start

# The server will start at http://localhost:3000
# The page will reload automatically when you make changes
```

### 2. Development Commands

```bash
# Start development server with specific port
npm start -- --port 3001

# Start development server with host binding
npm start -- --host 0.0.0.0

# Check for broken links in the documentation
npm run check-links

# Build the static site locally (for testing)
npm run build

# Serve the built site locally (for testing)
npm run serve
```

## Project Structure

### Key Directories

```
./
├── docs/                    # MDX content files organized by modules
│   ├── introduction/       # Introduction module content
│   ├── module-0-foundations/ # Physical AI Foundations
│   ├── module-1-ros2/      # ROS 2 Architecture
│   ├── module-2-simulation/ # Simulation Environments
│   ├── module-3-isaac/     # NVIDIA Isaac
│   ├── module-4-vla/       # Vision-Language-Action Robotics
│   └── capstone/          # Capstone project content
├── src/
│   ├── components/         # Reusable React components
│   │   ├── robotics/      # Robot visualization components
│   │   ├── simulators/    # Simulation components
│   │   └── interactive/   # Interactive learning components
│   ├── css/               # CSS modules and styling
│   └── utils/             # Utility functions
├── static/                # Static assets (images, data files)
├── .github/
│   └── workflows/         # CI/CD pipeline configurations
├── docusaurus.config.ts   # Docusaurus configuration
├── tsconfig.json          # TypeScript configuration
├── package.json           # Project dependencies and scripts
└── babel.config.js        # Babel configuration
```

## Creating Content

### 1. Adding a New Chapter

```bash
# Create a new chapter directory
mkdir docs/module-1-ros2/new-chapter

# Create the main MDX file
cat > docs/module-1-ros2/new-chapter/index.mdx << 'EOF'
---
title: New Chapter Title
description: Brief description of the chapter content
---

# New Chapter Title

This is the content of the new chapter.

## Learning Objectives

- Understand key concept 1
- Implement technique 2
- Apply knowledge to practical example

## Content

Your chapter content goes here. You can include:
- Text explanations
- Code examples
- Interactive components
- Images and diagrams

EOF
```

### 2. Creating Interactive Components

```bash
# Create a new React component for visualizations
mkdir -p src/components/robotics
cat > src/components/robotics/RobotVisualizer.tsx << 'EOF'
import React, { useState, useEffect } from 'react';
import * as THREE from 'three';

interface RobotVisualizerProps {
  robotModel?: string;
  parameters?: Record<string, any>;
}

export default function RobotVisualizer({
  robotModel = 'basic',
  parameters = {}
}: RobotVisualizerProps): JSX.Element {
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Initialize Three.js scene for robot visualization
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    const renderer = new THREE.WebGLRenderer();

    // Setup robot visualization
    // ... Three.js code to render robot

    setLoading(false);

    // Cleanup
    return () => {
      renderer.dispose();
    };
  }, [robotModel]);

  return (
    <div className="robot-visualizer-container">
      {loading ? (
        <div className="loading">Loading robot visualization...</div>
      ) : (
        <div id="robot-visualizer" className="visualizer-canvas" />
      )}
      <div className="controls">
        <p>Interactive controls for the robot visualization</p>
      </div>
    </div>
  );
}
EOF
```

### 3. Adding Code Examples

```bash
# Create a code example with explanation
cat > docs/module-1-ros2/ros-nodes/example-publisher.py << 'EOF'
#!/usr/bin/env python3
"""
ROS 2 Publisher Example
This example demonstrates how to create a simple publisher node.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
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
EOF
```

## Using Components in MDX

### 1. Importing Components

```mdx
---
title: Example Chapter
---

# Example Chapter

import RobotVisualizer from '@site/src/components/robotics/RobotVisualizer';

## Interactive Visualization

<RobotVisualizer robotModel="bipedal" />

This visualization shows a bipedal robot model with interactive controls.
```

### 2. Adding Exercises

```mdx
---
title: ROS 2 Exercises
---

# ROS 2 Exercises

## Exercise 1: Simple Publisher

Create a ROS 2 publisher node that publishes messages to a topic every 2 seconds.

```python
# Your solution here
```

<details>
<summary>Click to see solution</summary>

```python
# Solution code here
```

</details>
```

## Testing and Validation

### 1. Run Code Quality Checks

```bash
# Run TypeScript type checking
npx tsc --noEmit

# Run ESLint for code quality
npm run lint

# Run Prettier for code formatting
npm run format:check

# Run all tests
npm test
```

### 2. Build and Test

```bash
# Build the site
npm run build

# Test the built site locally
npm run serve

# Check for broken links
npm run check-links
```

### 3. Content Validation

```bash
# Validate MDX syntax
npx markdownlint docs/**/*.mdx

# Run accessibility checks
npx pa11y http://localhost:3000

# Check for spelling errors
npx cspell "**/*.{md,mdx,ts,tsx,js,jsx}"
```

## Deployment

### 1. GitHub Actions Setup

The project includes GitHub Actions workflows for CI/CD:

- `.github/workflows/ci.yml`: Runs on pull requests for linting, testing, and building
- `.github/workflows/deploy.yml`: Deploys to GitHub Pages on merge to main

### 2. Manual Deployment

```bash
# Deploy to GitHub Pages (requires proper configuration)
npm run deploy

# This command builds the site and pushes to the gh-pages branch
```

## Common Tasks

### 1. Update Dependencies

```bash
# Update npm packages to latest compatible versions
npm update

# Check for outdated packages
npm outdated

# Update to specific versions as needed based on research requirements
```

### 2. Add New Modules

```bash
# Create a new module directory
mkdir docs/module-5-advanced-topics

# Add the module to the sidebar configuration in docusaurus.config.ts
# Follow the pattern of existing modules
```

### 3. Update References

```bash
# When adding new academic references, ensure they follow APA/IEEE format
# Update the bibliography file with proper citations
# Verify all external links remain valid
```

## Troubleshooting

### 1. Common Issues

**Issue**: Development server won't start
**Solution**:
```bash
# Clear npm cache and reinstall dependencies
npm cache clean --force
rm -rf node_modules package-lock.json
npm install
npm start
```

**Issue**: TypeScript compilation errors
**Solution**: Check `tsconfig.json` settings and ensure all component props are properly typed

**Issue**: Interactive components not rendering
**Solution**: Verify Three.js and other visualization libraries are properly imported and initialized

### 2. Performance Issues

- If the development server is slow, try increasing Node.js memory limit:
  ```bash
  export NODE_OPTIONS="--max-old-space-size=4096"
  npm start
  ```

- For large visualization files, implement lazy loading and code splitting

### 3. Browser Compatibility

- Interactive 3D visualizations require modern browsers with WebGL support
- Provide fallbacks for older browsers as specified in the accessibility requirements
- Test on Chrome, Firefox, Safari, and Edge

## Next Steps

1. **Content Creation**: Start with the Introduction module and work through each core module
2. **Component Development**: Implement the required visualization and interaction components
3. **Testing**: Validate all content meets the educational and technical requirements
4. **Deployment**: Prepare for production deployment to GitHub Pages

For detailed information on specific aspects of development, refer to the complete implementation plan in `specs/1-physical-ai-textbook/plan/implementation-plan.md`.