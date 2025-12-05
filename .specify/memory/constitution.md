<!--
Version change: 1.0 -> 1.1
List of modified principles:
- Clarity: added target Flesch-Kincaid grade level.
- Source Use Requirements: added flexibility.
Added sections:
- Plagiarism Checking to Success Criteria
Removed sections:
- None
Templates requiring updates:
- plan-template.md: ✅ updated
- spec-template.md: ✅ updated
- tasks-template.md: ✅ updated
- commands/sp.constitution.md: ✅ updated
Follow-up TODOs: None
-->
# Constitution: Physical AI & Humanoid Robotics Textbook

**Document Type:** Constitution (Global Rules)
**Version:** 1.1
**Last Updated:** December 2025
**Status:** Active

---

## 1. Project Overview

### 1.1 Project Identity
**Title:** Textbook for Teaching Physical AI & Humanoid Robotics
**Development Method:** SpecKit-Plus + Claude Code + Docusaurus + GitHub Pages

### 1.2 Mission
Create a comprehensive, technically accurate, and pedagogically sound textbook for undergraduate engineering students and educators in Physical AI and Humanoid Robotics using AI-native, specification-driven development.

---

## 2. Core Principles

### 2.1 Spec-Driven Development

**PRINCIPLE:** Every chapter, section, and diagram must originate from a written specification using SpecKit-Plus.

**Implementation Rules:**
- ✅ No content creation without a corresponding specification document
- ✅ Every chapter requires `.claude/specs/chapter-XX-[name].md` before writing begins
- ✅ All diagrams must have specifications defining purpose, content, and format
- ✅ Code examples must be specified with inputs, outputs, and acceptance criteria
- ❌ "Just write something" approach is strictly prohibited

**Enforcement:**
```
Specification → Draft → Review → Final
     ↓
Without spec, no work proceeds
```

### 2.2 AI-Native Writing Workflow

**PRINCIPLE:** Claude Code is used for generation, refinement, and iterative improvement guided by specs.

**Primary Tools:**
- **Claude Code CLI** (90%): Content generation, code examples, iteration
- **Gemini CLI** (10%): Alternative perspectives, technical review
- **Context7 MCP**: Research, academic paper retrieval, citations
- **GitHub MCP**: Issue tracking, version control, deployment

**Workflow Standard:**
```bash
# Required process for every chapter:
1. Write specification
2. Generate content with Claude Code following spec
3. Review with Gemini CLI for alternative view
4. Refine with Claude Code based on feedback
5. Commit to GitHub with meaningful message
```

**Human Role:** Strategic direction, specification authoring, quality gates, final approval.

### 2.3 Technical Accuracy

**PRINCIPLE:** All information related to Physical AI, robotics engineering, humanoid kinematics, sensors, actuators, embedded systems, and safety guidelines must be validated against authoritative sources.

**Zero Hallucination Policy:**

❌ **Prohibited:**
- Invented robotics algorithms
- Fictional hardware specifications
- Unsourced mathematical formulas
- Made-up safety standards
- Non-existent research papers
- Fabricated software libraries

✅ **Required:**
- Every technical claim must cite authoritative source
- All formulas must be verifiable
- All hardware specs must match datasheets
- All safety claims must reference ISO/IEEE standards

**Verification Process:**
```
Technical Claim → Find Source → Add Citation → Verify → Include
                      ↓
                 No Source? → Remove or Mark for Research
```

### 2.4 Clarity for Students & Educators

**PRINCIPLE:** Content must be understandable for undergraduate-level engineering students while preserving scientific precision.

**Writing Standards:**
- **Target Audience:** 3rd-4th year engineering undergraduates
- **Prerequisites Assumed:** Linear algebra, calculus, Python programming, basic physics
- **Language:** Academic but accessible, avoid unnecessary jargon. Target a Flesch-Kincaid grade level of 10 or below.
- **Explanation Depth:** Build from fundamentals, explain "why" not just "what"

**Pedagogical Requirements:**
- Every concept introduced with clear definition
- Complex topics broken into digestible sections
- Technical terms defined on first use
- Mathematical notation explained before use
- Examples progress from simple to complex

**Educator Support:**
- Clear learning objectives per chapter
- Ready-to-use teaching materials
- Lab instructions with expected outcomes
- Assessment rubrics provided

### 2.5 Hands-On Orientation

**PRINCIPLE:** Every chapter must contain practical exercises, simulations, code samples, and robotics labs.

**Mandatory Practical Components per Chapter:**

| Component | Minimum Quantity | Purpose |
|-----------|-----------------|---------|
| **Worked Examples** | 2 | Step-by-step demonstrations |
| **Code Samples** | 2 | Reproducible implementations (Python/ROS2/C++) |
| **Simulation Labs** | 1 | Safe experimentation environment |
| **Practice Problems** | 5 | Reinforce concepts (varied difficulty) |
| **Mini-Project** | 1 | Apply chapter knowledge |

**Lab Infrastructure:**
- All labs must run in: Gazebo, Webots, PyBullet, or Isaac Sim
- Code must be Python 3.8+, ROS2 Humble+
- Optional hardware: TurtleBot3, OpenManipulator, or equivalent
- Every code sample must include setup instructions
- Every lab must have expected outputs/screenshots

**Reproducibility Standard:**
```
A student should be able to:
1. Read the lab instructions
2. Run the provided code
3. Achieve the stated outcomes
4. Without external help or debugging
```

### 2.6 Ethical & Safe Robotics

**PRINCIPLE:** All content must comply with global safety best practices in robotics and AI alignment principles.

**Safety Compliance:**
- ✅ All humanoid robotics instructions follow ISO 10218 (Industrial Robots Safety)
- ✅ All collaborative robotics content follows ISO/TS 15066 (Collaborative Robots)
- ✅ All robot definitions align with ISO 8373 (Robotics Vocabulary)
- ❌ No unsafe configurations or practices included
- ❌ No instructions that could lead to injury or property damage

**Ethical Guidelines:**
Every chapter addressing AI/autonomy must include:
- Potential societal impacts discussion
- Ethical considerations in deployment
- Bias and fairness concerns
- Privacy and data protection
- Human oversight requirements

**Safety Reminders:**
```markdown
⚠️ **Safety Note:** All chapters involving physical robots must include:
- Emergency stop procedures
- Safe operating distances
- Risk assessment guidelines
- PPE recommendations where applicable
```

---

## 3. Key Standards

### 3.1 Traceability

**All Technical Content Must Be Traceable:**

| Content Type | Traceability Requirement |
|--------------|-------------------------|
| **Definitions** (PID control, inverse kinematics, sensor fusion) | Reference known academic or industrial standard |
| **Formulas** (kinematics equations, control laws) | Must be verifiable in cited source |
| **Algorithms** (path planning, SLAM) | Cite original paper or established textbook |
| **Hardware Specs** (servo torque, sensor range) | Link to manufacturer datasheet |
| **Safety Standards** | Cite specific ISO/IEEE standard number and section |

**Citation Standard:**
```markdown
The Denavit-Hartenberg convention [1] provides a systematic method for...

[1] J. Denavit and R. S. Hartenberg, "A kinematic notation for lower-pair
    mechanisms based on matrices," ASME Journal of Applied Mechanics, 1955.
```

### 3.2 Style & Structure

**Markdown Compatibility:**
- ✅ Clean Docusaurus-compatible markdown only
- ✅ Use Docusaurus admonitions (:::note, :::warning, :::tip)
- ✅ Code blocks with language specification (```python, ```cpp, ```bash)
- ✅ Math equations using KaTeX syntax ($inline$ and $display$)

**Content Pipeline:**
```
spec → draft → review → final
  ↓      ↓       ↓       ↓
 Plan  Write  Verify  Publish
```

**File Organization:**
```
/docs/
  /part1-foundations/
    /chapter-01-intro.md
    /chapter-02-math.md
  /part2-perception/
    ...
/static/
  /img/
    /chapter-01/
    /chapter-02/
/examples/
  /chapter-01/
    simulation.py
    exercise-solution.py
```

**Code Examples Standard:**
- Must be reproducible (include all imports, dependencies)
- Must include setup instructions
- Must specify versions (Python 3.10, ROS2 Humble, etc.)
- Must include expected output or behavior
- Should follow PEP 8 (Python) or Google C++ Style Guide

### 3.3 Source Use Requirements

**Source Distribution per Chapter:**

| Source Type | Minimum % | Example |
|-------------|-----------|---------|
| **Academic** | 30% | Robotics textbooks, IEEE papers, conference proceedings |
| **Industry/Standards** | 30% | ROS docs, ISO standards, manufacturer specs, technical reports |
| **AI-Generated** | Maximum 20% | Must have human technical correction and verification |
| **Other** | 0% | Blog posts, non-peer reviewed articles, etc.|

This distribution is a target, not a hard requirement, and should be justified in each chapter's specification.

**Academic Sources (Tier 1):**
- Established textbooks: Siciliano, Craig, Spong, Murray, Thrun
- Peer-reviewed journals: IEEE T-RO, IJRR, Autonomous Robots
- Top conferences: ICRA, IROS, RSS, CoRL

**Industry/Standards Sources (Tier 2):**
- ISO robotics standards (10218, TS 15066, 8373)
- Official ROS/ROS2 documentation
- Hardware manufacturer datasheets
- Open-source projects: PyBullet, MuJoCo, Drake

**AI-Generated Content Rules:**
- Cannot be primary source of technical information
- Must be verified by human with domain expertise
- Must be cross-referenced with authoritative sources
- Used for synthesis and explanation, not novel claims

### 3.4 Diagrams & Figures

**Creation Process:**
```
1. Claude Code generates ASCII/text draft
2. Convert to Mermaid diagram (for simple flowcharts/graphs)
3. OR create in draw.io/Excalidraw for complex diagrams
4. Export final as SVG (preferred) or high-res PNG
5. Store in /static/img/chapter-XX/
```

**Diagram Standards:**
- ✅ Vector format (SVG) preferred for scalability
- ✅ All text must be readable at 100% zoom
- ✅ Color-blind friendly palette (use ColorBrewer)
- ✅ Alt text for accessibility
- ✅ Caption with figure number and description

**Figure Naming Convention:**
```
/static/img/chapter-03/fig-03-01-dh-parameters.svg
/static/img/chapter-03/fig-03-02-forward-kinematics-flow.svg
```

### 3.5 Repository Standards

**Version Control:**
- ✅ All content stored in GitHub repository
- ✅ Meaningful commit messages following conventional commits
- ✅ Branch strategy: `main` (production), `develop` (staging), `chapter-XX` (work)
- ✅ Pull requests required for all changes to `main`

**Commit Message Format:**
```
type(scope): description

Examples:
docs(chapter-03): add inverse kinematics derivation
fix(chapter-05): correct PID formula citation
feat(labs): add Gazebo simulation for bipedal walking
```

**Deployment:**
- ✅ GitHub Pages hosting via Docusaurus build
- ✅ Automated CI/CD pipeline (GitHub Actions)
- ✅ Build must pass before merge to `main`
- ✅ Automated link validation on every commit
- ✅ Automated spell-check and markdown linting

**CI/CD Requirements:**
```yaml
# Must pass:
- npm run build (Docusaurus build succeeds)
- npm run lint (markdown linting)
- npm run check-links (no broken links)
- npm run spell-check (no typos)
```

---

## 4. Constraints

### 4.1 Book Length

| Parameter | Specification |
|-----------|--------------|
| **Minimum Chapters** | 12 |
| **Maximum Chapters** | 20 |
| **Words per Chapter** | 1,500 – 2,500 |
| **Total Book Length** | 18,000 – 50,000 words |

**Chapter Structure:**
- Introduction: 200-300 words
- Main Content: 1,000-1,800 words
- Practical Exercises: 200-300 words
- Summary & References: 100-200 words

### 4.2 Technical Coverage Required

**Mandatory Topics (Must Cover):**

1. **Physical AI Foundations**
   - Embodied intelligence concepts
   - Perception-action loops
   - Sensor-motor coordination
   - Learning in physical systems

2. **Humanoid Robotics Hardware**
   - Mechanical design principles
   - Degrees of freedom and kinematic chains
   - Actuator types (motors, hydraulics, pneumatics)
   - Structural materials and considerations

3. **Perception Systems**
   - Vision (cameras, depth sensors, LiDAR)
   4.  Inertial Measurement Units (IMUs)
   - Force/torque sensors
   - Tactile sensors
   - Sensor fusion algorithms

4. **Control Systems**
   - PID control (theory and tuning)
   - Model Predictive Control (MPC)
   - Reinforcement Learning-based control
   - Whole-body control
   - Balance and stability control

5. **Actuation & Locomotion**
   - Bipedal walking (ZMP, CPG approaches)
   - Gait planning and optimization
   - Terrain adaptation
   - Energy efficiency

6. **Simulation**
   - Gazebo Classic and Gazebo Garden
   - Webots
   - Isaac Sim (NVIDIA)
   - PyBullet basics
   - Simulation-to-reality transfer

7. **Safety & Ethics**
   - ISO safety standards compliance
   - Risk assessment methodologies
   - Human-robot interaction safety
   - Ethical considerations in humanoid deployment
   - AI alignment and control

8. **Hands-On Labs**
   - Every theoretical chapter paired with practical lab
   - Progressive complexity (simple → full humanoid)
   - Simulation-first, hardware-optional approach

### 4.3 Tooling Constraints

**Specification Format:**
- ✅ All specifications use SpecKit-Plus standard format
- ✅ Five-phase structure: Constitution → Specification → Planning → Task → Implementation
- ✅ Stored in `.claude/specs/` directory
- ✅ Markdown format with required sections

**Content Generation:**
- ✅ All writing exclusively through Claude Code CLI workflows
- ✅ No manual writing of technical content (human only writes specs and reviews)
- ✅ Iterative refinement using AI feedback loops

**Formatting:**
- ✅ Must remain Docusaurus-compatible at all times
- ✅ No custom React components without specification
- ✅ Standard markdown + Docusaurus features only (admonitions, tabs, code blocks)

---

## 5. Success Criteria

- [ ] Use plagiarism detection tools on all generated content.

### 5.1 Documentation Quality

**Specification Compliance:**
- [ ] All chapters meet their written specification 100%
- [ ] No content missing from defined requirements
- [ ] All sections present as specified
- [ ] Word count within specified range (±10%)

**Completeness:**
- [ ] Zero placeholder text ("TODO", "TBD", "[Add content here]")
- [ ] All diagrams rendered correctly on GitHub Pages
- [ ] All code examples included and properly formatted
- [ ] All references accessible and correctly formatted

**Navigation & Usability:**
- [ ] Table of contents accurate and complete
- [ ] All internal links functional
- [ ] Search functionality works across all chapters
- [ ] Mobile-responsive layout verified

### 5.2 Technical Accuracy

**Verification Checklist:**
- [ ] All robotics equations verified against source material
- [ ] No hallucinated hardware or algorithms present
- [ ] All factual claims sourced and cited
- [ ] All formulas dimensionally correct
- [ ] All code examples tested and functional
- [ ] All safety claims validated against ISO standards (10218, TS 15066)

**Source Quality:**
- [ ] 40%+ academic sources (peer-reviewed, established textbooks)
- [ ] 40%+ industry/standards sources (ISO, ROS docs, datasheets)
- [ ] AI-generated content ≤20% and verified by human expert

### 5.3 Pedagogical Quality

**Student Success:**
- [ ] Students can complete labs without external assistance
- [ ] All code examples run on first attempt with provided setup
- [ ] Practice problems solvable with chapter content alone
- [ ] Progressive difficulty curve validated (pilot testing)

**Educator Success:**
- [ ] Educators can teach from the textbook without gaps
- [ ] All learning objectives measurable and achievable
- [ ] Assessment materials provided for every chapter
- [ ] Suggested teaching timeline included

**Engagement:**
- [ ] Hands-on component in every chapter
- [ ] Real-world examples and applications included
- [ ] Visual aids support comprehension (1-3 diagrams per chapter minimum)

### 5.4 Book Integrity

**Technical Infrastructure:**
- [ ] Zero broken links in Docusaurus deployment
- [ ] Zero unresolved placeholders in published version
- [ ] CI/CD pipeline passes for every commit
- [ ] Automated tests green (build, lint, link-check, spell-check)

**Deployment:**
- [ ] GitHub Pages live and accessible
- [ ] All images load correctly
- [ ] All code snippets syntax-highlighted
- [ ] Search indexing complete and functional

**Version Control:**
- [ ] All commits follow conventional commit format
- [ ] Meaningful commit history (not "fix", "update", etc.)
- [ ] No sensitive data in repository
- [ ] README complete with setup instructions

### 5.5 Ethics & Safety

**Safety Compliance:**
- [ ] All humanoid robotics instructions follow ISO 10218 (Industrial Robots)
- [ ] All collaborative robot content follows ISO/TS 15066
- [ ] Robot definitions align with ISO 8373 (Robotics Vocabulary)
- [ ] Emergency stop procedures included in hardware labs
- [ ] Risk assessment guidelines provided

**Ethical Standards:**
- [ ] No unsafe or harmful robotics configurations included
- [ ] AI alignment principles discussed where relevant
- [ ] Bias and fairness considerations addressed
- [ ] Privacy implications of perception systems explained
- [ ] Societal impact discussions included in relevant chapters

**Content Safety:**
- [ ] No instructions that could lead to injury
- [ ] All high-risk operations clearly marked with warnings
- [ ] PPE recommendations included where applicable
- [ ] Safe operating procedures documented

---

## 6. Enforcement & Governance

### 6.1 Constitution Authority

**This constitution is:**
- ✅ The highest-level governing document
- ✅ Immutable except for major project pivots (requires team consensus)
- ✅ Referenced by all specifications, plans, and tasks
- ✅ The standard against which all work is judged

### 6.2 Violation Handling

**If work violates this constitution:**
1. **Halt:** Stop the violating work immediately
2. **Review:** Determine which principle was violated
3. **Correct:** Rewrite/redo following constitution
4. **Learn:** Update specifications or processes to prevent recurrence

**Common Violations:**
- Creating content without specification → Reject, write spec first
- Unsourced technical claims → Remove or cite
- Broken code examples → Fix or remove
- Unsafe procedures → Rewrite with safety compliance

### 6.3 Amendment Process

**To modify this constitution:**
1. Propose change in GitHub Issue with `constitution-amendment` label
2. Justify why amendment is necessary (not just convenient)
3. Review impact on existing specifications and work
4. Require consensus (all primary contributors agree)
5. Update version number and last updated date
6. Communicate change to all contributors

**Amendments require extraordinary justification:**
- Major technological shift (e.g., Docusaurus deprecated)
- Fundamental change in target audience
- New safety standards supersede old ones
- Project scope expansion beyond initial vision

---

## 7. Quick Reference

### 7.1 Daily Development Checklist

Before starting work:
- [ ] Specification exists for what I'm building
- [ ] I understand the acceptance criteria
- [ ] I have authoritative sources ready for citations

During work:
- [ ] Following SpecKit-Plus five-phase process
- [ ] Using Claude Code CLI for content generation
- [ ] Citing all technical claims
- [ ] Testing all code examples

Before committing:
- [ ] Content meets specification 100%
- [ ] No placeholder text remains
- [ ] All links functional
- [ ] Commit message follows conventional format

### 7.2 Core Principles Summary

1. **Spec-Driven Development** → No work without specification
2. **AI-Native Workflow** → Claude Code is primary tool
3. **Technical Accuracy** → Every claim must be verifiable
4. **Clarity** → Undergraduate-level, pedagogically sound
5. **Hands-On** → Every chapter has practical components
6. **Ethics & Safety** → ISO compliance, no harmful content

---

**End of Constitution**

*This document governs all development of the Physical AI & Humanoid Robotics Textbook. All specifications, plans, tasks, and implementations must align with these principles.*