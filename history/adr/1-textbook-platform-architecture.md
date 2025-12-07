# ADR-1: Textbook Platform Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** physical-ai-textbook
- **Context:** Need to establish a robust, scalable platform for delivering interactive educational content on Physical AI and Humanoid Robotics that supports both static content and complex 3D visualizations.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- **Framework**: Docusaurus 3.x (React-based static site generator)
- **Content Format**: MDX (Markdown + JSX) for interactive content
- **Components**: React 18 + TypeScript 5.0 (.tsx files)
- **Styling**: CSS Modules + Tailwind CSS
- **Visualization**: Three.js for 3D robotics visualizations, D3.js for data visualization
- **Deployment**: GitHub Pages with Cloudflare CDN
- **CI/CD**: GitHub Actions for automated builds and deployments
- **Development Tools**: Claude Code for content generation, MCP integration for research sync

## Consequences

### Positive

- Static site generation provides excellent performance and SEO
- Docusaurus provides built-in features like versioning, search, dark mode, i18n
- MDX allows embedding interactive React components directly in content
- TypeScript provides type safety for complex interactive components
- GitHub Pages provides free, reliable hosting with CDN
- Three.js enables rich 3D visualizations for robotics concepts
- Component reusability supports consistent educational experience

### Negative

- Learning curve for authors to use MDX and React components
- Complexity of managing 3D visualizations and their performance across devices
- Dependency on GitHub ecosystem for deployment
- Static site limitations for real-time collaboration features
- Potential performance issues with complex 3D visualizations on lower-end hardware

## Alternatives Considered

Alternative A: Custom React application with Next.js + Three.js
- Pros: More flexibility, better performance optimization, real-time features
- Cons: More complex setup, higher maintenance, less educational-focused features

Alternative B: Traditional LMS platform (Moodle, Canvas)
- Pros: Built-in assessment tools, user management, pedagogical features
- Cons: Less flexible for custom visualizations, potential licensing costs, less open-source aligned

Alternative C: Static site with plain HTML/CSS/JS
- Pros: Simpler technology stack, no framework dependencies
- Cons: Less interactive capability, no component reusability, harder to maintain

## References

- Feature Spec: specs/1-physical-ai-textbook/spec.md
- Implementation Plan: specs/1-physical-ai-textbook/plan/implementation-plan.md
- Research Document: specs/1-physical-ai-textbook/plan/research.md
- Evaluator Evidence: history/prompts/physical-ai-textbook/3-physical-ai-textbook-plan.plan.prompt.md