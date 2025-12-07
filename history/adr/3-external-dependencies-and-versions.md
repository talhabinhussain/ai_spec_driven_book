# ADR-3: External Dependencies and Technology Versions

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** physical-ai-textbook
- **Context:** Need to select stable, well-supported versions for core technologies and external dependencies that ensure long-term maintainability, compatibility with the robotics ecosystem, and consistency for students and educators.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- **ROS 2**: Humble Hawksbill (LTS version, supported until 2027)
- **Gazebo**: Garden (stable version with good ROS 2 integration)
- **Isaac Sim**: 2023.2.1 (stable version with good documentation)
- **Unity**: 2022.3 LTS (long-term support version)
- **Review Cycle**: Quarterly review cycles for dependency updates as specified in clarifications
- **Stability Focus**: Prioritize long-term support and stability over cutting-edge features

## Consequences

### Positive

- LTS versions provide 5-year support cycles ensuring long-term stability
- Good community support and extensive documentation for selected versions
- Compatibility with existing robotics ecosystem and educational resources
- Reduced maintenance burden with stable, well-tested versions
- Consistency across installations for students and educators
- Quarterly review cycles ensure technology remains current without constant churn

### Negative

- Potentially missing out on newer features and improvements
- Possible compatibility issues with newer tools or libraries
- May require additional effort to maintain compatibility over time
- Students may need to adapt to newer versions when entering industry
- Some cutting-edge robotics research may not be compatible with LTS versions

## Alternatives Considered

Alternative A: Latest available versions (e.g., ROS 2 Iron Irwini)
- Pros: Access to newest features and improvements
- Cons: Shorter support cycle, potential instability, more frequent updates required

Alternative B: Previous stable versions (e.g., ROS 2 Foxy)
- Pros: Proven stability, extensive usage history
- Cons: Approaching end-of-life, missing important improvements, less community support

Alternative C: Mixed version approach (different lifecycle per tool)
- Pros: Could optimize each tool for its specific needs
- Cons: Increased complexity in management, testing, and documentation

Alternative D: No formal review cycle for updates
- Pros: Reduced maintenance overhead initially
- Cons: Risk of technology becoming obsolete, security vulnerabilities, compatibility issues

## References

- Feature Spec: specs/1-physical-ai-textbook/spec.md
- Implementation Plan: specs/1-physical-ai-textbook/plan/implementation-plan.md
- Research Document: specs/1-physical-ai-textbook/plan/research.md
- Evaluator Evidence: history/prompts/physical-ai-textbook/3-physical-ai-textbook-plan.plan.prompt.md