---
id: 3
title: "physical-ai-textbook-plan"
stage: "plan"
date_iso: "2025-12-06"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "physical-ai-textbook"
branch: "1-physical-ai-textbook"
user: "user"
command: "/sp.plan"
labels: ["planning", "textbook", "robotics", "ai", "education"]
links:
  spec: "specs/1-physical-ai-textbook/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - specs/1-physical-ai-textbook/plan/implementation-plan.md
  - specs/1-physical-ai-textbook/plan/research.md
  - specs/1-physical-ai-textbook/plan/data-model.md
  - specs/1-physical-ai-textbook/plan/quickstart.md
  - specs/1-physical-ai-textbook/plan/contracts/textbook-api.yaml
tests_yaml: |
  - Implementation plan completeness check
  - Research document validation
  - Data model verification
  - Quickstart guide testing
  - API contract validation
---

# Implementation Plan for Physical AI & Humanoid Robotics Textbook

## User Input (PROMPT_TEXT)

# Teaching Physical AI & Humanoid Robotics — Book Plan

## 1 — Purpose

I am writing an open-source textbook on **Physical AI & Humanoid Robotics** with hands-on labs, runnable code, and live research updates. The book uses research-concurrent authoring and deploys via Docusaurus → GitHub Pages. Citations in APA style.

## 2 — Core Tech Stack

**Documentation**: Docusaurus (React-based static site generator)

- **Content**: MDX (Markdown + JSX) for interactive content
- **Components**: React 18 + TypeScript 5.0 (.tsx files)
- **Styling**: CSS Modules + Tailwind CSS
- **Features**: Versioning, search, dark mode, i18n

**Development**:

- **Claude Code**: AI-assisted content generation and code examples
- **TypeScript/JavaScript**: All code examples and implementations
- **React**: Interactive components, visualizations, simulations
- **Docker**: Reproducible environments (optional for deployment)

**MCP Integration**:

- **Context7 MCP**: Live research sync, bibliography updates, note integration
- **GitHub MCP**: Automated PR creation, deployment triggers, version control

**Deployment**:

- **GitHub Actions**: CI/CD pipelines (.github/workflows/)
- **GitHub Pages**: Free static hosting with CDN + HTTPS

**Quality Tools**: ESLint, Prettier, Markdownlint, Jest, React Testing Library, Lighthouse

**Build Flow**:

```
Research (Context7 MCP) → Draft (MDX + TSX) → Components (React + TypeScript) →
GitHub MCP → CI (Actions) → Build (Docusaurus) → Deploy (Pages)
```

## 3 — Implementation Phases (12 months)

**Phase 0: Setup** (Month 0-1)

- Docusaurus + React + TypeScript setup in current directory
- GitHub repo with Actions CI/CD
- Context7 + GitHub MCP integration
- Sample MDX chapter with .tsx components
- Licenses: CC-BY-4.0 (text), MIT (code)

**Phase 1: Research** (Months 1-3, per chapter)

- Literature review via Context7 MCP (APA/BibTeX)
- Learning objectives and chapter outlines
- Interactive component sketches
- Code example prototypes in TypeScript

**Phase 2: Foundation** (Months 3-6)

- Core chapters: Kinematics, Control, Perception
- MDX content + React/TypeScript visualizations
- Interactive demos with live code examples
- First GitHub Pages deployment

**Phase 3: Analysis** (Months 6-9)

- Advanced simulations in TypeScript (physics engines)
- Advanced React components (Three.js, D3.js, WebGL)
- Interactive benchmarks and comparisons
- Dataset visualization interfaces

**Phase 4: Publication** (Months 9-12)

- Complete lab modules (6-8) with interactive exercises
- Capstone projects with rubrics
- Assessment quizzes (React components)
- v1.0 release with DOI
- Full site deployment

## 4 — Component Architecture

**Content (MDX + React + TypeScript)**

- MDX files in `docs/` with frontmatter
- React components in `src/components/` (.tsx)
- TypeScript interfaces for type safety
- CSS modules for styling
- All code examples in TypeScript/JavaScript
- Interactive code playgrounds embedded in MDX

**CI/CD (GitHub Actions)**

- `ci.yml`: Lint, test, build on PR
  - ESLint + Prettier for TypeScript/React
  - Markdownlint for MDX
  - TypeScript type checking
  - Docusaurus build test
  - Jest unit tests
- `deploy.yml`: Auto-deploy to GitHub Pages on merge
  - Build production Docusaurus site
  - Deploy to `gh-pages` branch
  - Verify deployment

**Deployment (GitHub Pages)**

1. Push to `main` → GitHub Actions trigger
2. Docusaurus builds static site → `build/`
3. Deploy to `gh-pages` branch
4. GitHub Pages auto-serves with HTTPS
5. GitHub MCP notifies deployment status

**MCP Workflow**

- Context7: Research updates → trigger rebuild
- GitHub: Auto-create PRs, manage branches
- Real-time sync keeps content current

## 5 — Per-Chapter Workflow

1. **Outline** (1-2 days): MDX file + learning objectives + React component needs
2. **Research** (1-2 weeks): Context7 MCP for literature + APA bibliography
3. **Draft** (3-5 days): Claude Code assists MDX content + code placeholders
4. **Code Examples** (1 week): Implement algorithms in TypeScript + live demos
5. **Components** (3-5 days): Build React/TypeScript visualizations + tests
6. **Revise** (2-3 days): Integrate results + embed components in MDX
7. **Review** (1 week): Peer + pedagogy + accessibility validation
8. **Publish** (2-3 days): CI checks → GitHub MCP PR → merge → auto-deploy

**Principle**: Research-concurrent (per chapter), iterate: draft → code → build → revise

## 6 — Key Decisions

**Why Docusaurus + MDX?**

- React-based with TypeScript component support
- Embed JSX directly in Markdown (.mdx)
- Built-in versioning, search, dark mode
- Static export perfect for GitHub Pages

**Why React + TypeScript Only?**

- Single language for entire project (consistency)
- Type-safe components (.tsx files)
- Better IDE support and refactoring
- All simulations and visualizations in browser
- No separate runtime environments needed
- Interactive code playgrounds with live execution

**Why GitHub Pages?**

- Free hosting with CDN and HTTPS
- Direct GitHub integration
- Easy rollback via git history
- Automated deployment with Actions

**Why MCPs?**

- Context7: Real-time research synchronization
- GitHub: Automated version control and deployment
- Keeps content current without manual triggers

**Stack Choices**:

- **TypeScript**: Type safety, single language, browser-native
- **React**: Component reusability, interactive visualizations
- **Three.js**: 3D robotics visualizations in browser
- **D3.js**: Data visualization and interactive charts
- **Docker**: Optional for consistent build environments

## 7 — Quality Checklist

**Content**

- [ ] MDX with frontmatter and learning objectives
- [ ] APA bibliography via Context7 MCP
- [ ] All claims referenced

**Code**

- [ ] TypeScript code examples type-safe
- [ ] All algorithms implemented in TS/JS
- [ ] Interactive demos functional
- [ ] CI smoke test passes (<5 min)

**Components**

- [ ] React/TypeScript (.tsx) components type-safe
- [ ] Responsive (mobile + desktop)
- [ ] Jest unit tests pass
- [ ] Accessibility verified
- [ ] Performance optimized (Lighthouse)

**Deployment**

- [ ] Docusaurus builds successfully
- [ ] GitHub Actions workflows pass
- [ ] GitHub Pages deployment live
- [ ] All links functional

## 8 — Minimal Viable Book (M0)

**6 Chapters**:

1. Introduction (MDX + React components)
2. Kinematics (TypeScript algorithms + React visualizer)
3. Dynamics (Physics simulation in Three.js)
4. Control (Interactive parameter tuning in React)
5. Perception (WebGL visualizations + data processing)
6. Sim-to-Real (Complete experiment with TypeScript)

**Per Chapter**:

- 5-10 page MDX file
- TypeScript code examples (embedded in MDX)
- 1 hands-on interactive lab (React component)
- 1 quiz (React component with validation)
- 1-2 custom .tsx visualization components

**Infrastructure**:

- Docusaurus site with custom theme
- GitHub Actions CI/CD (2 workflows)
- Context7 + GitHub MCP integrated
- GitHub Pages deployment (live URL)
- 10+ React/TypeScript components
- Interactive code playground system

**Timeline**: 3-4 months from setup to M0 release

## 9 — Next Steps

**Note**: All initialization will happen in the **current/root directory** where Spec-Kit-Plus and Claude Code are already configured.

**1. Initialize Docusaurus in Current Directory**

```bash
# Initialize Docusaurus + TypeScript in root directory (.)
npx create-docusaurus@latest . classic --typescript --skip-install

# Install dependencies
npm install

# Install additional packages for robotics visualizations
npm install three @types/three d3 @types/d3 chart.js react-chartjs-2

# Install dev dependencies
npm install --save-dev @testing-library/react @testing-library/jest-dom \
  eslint prettier eslint-config-prettier jest
```

**2. Project Structure (in root directory)**

```
./                       # Current/root directory
├── docs/               # MDX content files
│   ├── intro/
│   ├── kinematics/
│   ├── control/
│   └── perception/
├── src/
│   ├── components/     # React/TypeScript (.tsx)
│   │   ├── robotics/  # Robot visualizers
│   │   ├── simulators/ # Physics simulations
│   │   └── interactive/ # Interactive demos
│   ├── css/           # CSS modules
│   └── utils/         # TypeScript utility functions
├── static/            # Static assets, datasets
├── .github/
│   └── workflows/     # CI/CD pipelines
├── docusaurus.config.ts
├── tsconfig.json
└── package.json
```

**3. Configure GitHub Actions**

Create in `.github/workflows/`:

- `ci.yml`: Lint, type-check, test, build on PR
- `deploy.yml`: Auto-deploy to GitHub Pages on merge to main

**4. Setup MCPs (in current directory)**

- Context7 MCP: Configure research sync topics
- GitHub MCP: Setup auto-deployment triggers
- Create demo integration in first chapter

**5. Create First Chapter**

```bash
# Create intro chapter
mkdir -p docs/intro
cat > docs/intro/index.mdx << 'EOF'
---
title: Introduction
description: Getting started with Physical AI & Humanoid Robotics
---

# Introduction to Physical AI

import RobotVisualizer from '@site/src/components/robotics/RobotVisualizer';

<RobotVisualizer />

## What You'll Learn
- Fundamentals of robotics and AI
- Interactive simulations and visualizations
- Hands-on implementation in TypeScript
EOF

# Create sample React component
mkdir -p src/components/robotics
cat > src/components/robotics/RobotVisualizer.tsx << 'EOF'
import React from 'react';

export default function RobotVisualizer(): JSX.Element {
  return (
    <div style={{ padding: '2rem', border: '1px solid #ccc' }}>
      <h3>Robot Visualizer</h3>
      <p>Interactive 3D robot visualization will appear here</p>
    </div>
  );
}
EOF
```

**6. Configure TypeScript**

Ensure `tsconfig.json` includes:

```json
{
  "extends": "@docusaurus/tsconfig",
  "compilerOptions": {
    "jsx": "react",
    "strict": true,
    "esModuleInterop": true
  }
}
```

**7. Document in Root**

- `./README.md`: Project overview and setup
- `./CONTRIBUTING.md`: MDX + TypeScript authoring guide
- `./docs/decisions.md`: Architecture decisions
- `./LICENSE`: CC-BY-4.0 and MIT dual license

**8. Verify Setup**

```bash
# Test Docusaurus development server
npm start

# Run type checking
npx tsc --noEmit

# Run linting
npm run lint

# Test build
npm run build

# Preview production build
npm run serve
```

**Priority**: Initialize in current dir → Docusaurus + TypeScript → Components → CI/CD → MCPs → First chapter → Deploy

**Next Actions with Claude Code**:

1. Run initialization commands
2. Create sample MDX chapter with React component
3. Setup GitHub Actions workflows
4. Configure MCP integrations
5. Build first interactive visualization
6. Deploy to GitHub Pages

## Assistant Response (RESPONSE_TEXT)

Successfully completed the implementation planning for the Physical AI & Humanoid Robotics Textbook project. The following key artifacts were created:

1. **Implementation Plan** (specs/1-physical-ai-textbook/plan/implementation-plan.md): Comprehensive plan outlining the system architecture, technology stack, development phases, and success criteria.

2. **Research Document** (specs/1-physical-ai-textbook/plan/research.md): Resolved all "NEEDS CLARIFICATION" items from the original requirements, including specific version requirements for ROS 2, Gazebo, and Isaac Sim, visualization requirements, performance metrics, accessibility requirements, and CDN strategy.

3. **Data Model** (specs/1-physical-ai-textbook/plan/data-model.md): Detailed entity definitions for all textbook components including Chapters, LearningObjectives, Exercises, Assessments, Visualizations, CodeExamples, References, and HardwareSpecifications with their relationships and validation rules.

4. **Quickstart Guide** (specs/1-physical-ai-textbook/plan/quickstart.md): Complete guide for setting up the development environment, creating content, and deploying the textbook platform.

5. **API Contracts** (specs/1-physical-ai-textbook/plan/contracts/textbook-api.yaml): OpenAPI specification for the educational platform components, defining endpoints for chapters, exercises, assessments, visualizations, and search functionality.

The plan aligns with the original requirements and constitution, ensuring:
- Spec-driven development approach
- AI-native writing workflow with Claude Code
- Technical accuracy with verifiable claims
- Clarity for students and educators
- Hands-on orientation with interactive components
- Ethical and safe robotics education

All critical decisions have been documented and validated against the project constitution. The implementation plan provides a clear roadmap for developing the Docusaurus-based textbook platform with interactive components for teaching Physical AI and Humanoid Robotics.

## Outcome

The implementation plan is complete and ready for the next phase of development. The Physical AI & Humanoid Robotics Textbook project now has a solid foundation with clear technical architecture, development workflow, and success criteria defined.