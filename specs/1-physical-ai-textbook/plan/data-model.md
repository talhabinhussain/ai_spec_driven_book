# Data Model: Physical AI & Humanoid Robotics Textbook

**Feature**: Physical AI & Humanoid Robotics Textbook
**Branch**: `1-physical-ai-textbook`
**Created**: 2025-12-06
**Status**: Complete
**Author**: Claude Code

---

## Entity Definitions

### Chapter
A self-contained educational module that covers specific aspects of physical AI and humanoid robotics.

**Fields**:
- `id` (string): Unique identifier for the chapter (e.g., "module-0-foundations")
- `title` (string): Display title of the chapter
- `description` (string): Brief description of chapter content
- `moduleNumber` (number): Sequential number of the module (0-4 for core modules)
- `duration` (number): Estimated completion time in hours
- `prerequisites` (string[]): List of prerequisite knowledge or chapters
- `learningObjectives` (LearningObjective[]): Array of learning objectives
- `content` (string): Main content in MDX format
- `exercises` (Exercise[]): Array of practical exercises
- `assessments` (Assessment[]): Array of assessment components
- `visualizations` (Visualization[]): Array of interactive visualizations
- `codeExamples` (CodeExample[]): Array of code examples
- `references` (Reference[]): Array of academic and technical references
- `createdAt` (string): Creation timestamp
- `updatedAt` (string): Last update timestamp

**Validation Rules**:
- `id` must be unique across all chapters
- `title` must be 5-100 characters
- `moduleNumber` must be 0-4 for core modules, 5+ for advanced
- `duration` must be between 2-10 hours
- `learningObjectives` array must contain 2-5 items
- `exercises` array must contain 1-3 items
- `assessments` array must contain 0-2 items

### LearningObjective
A measurable outcome that students should achieve after completing a module or section.

**Fields**:
- `id` (string): Unique identifier within the chapter
- `description` (string): Clear, measurable statement of what students will learn
- `verificationMethod` (string): How the objective will be verified (exercise, assessment, etc.)
- `difficultyLevel` (string): "beginner", "intermediate", or "advanced"
- `relatedExercises` (string[]): IDs of exercises that verify this objective

**Validation Rules**:
- `description` must be 10-100 characters
- `difficultyLevel` must be one of the allowed values
- `verificationMethod` must reference an existing exercise or assessment

### Exercise
A hands-on activity that allows students to apply theoretical concepts to real-world scenarios.

**Fields**:
- `id` (string): Unique identifier within the chapter
- `title` (string): Descriptive title of the exercise
- `description` (string): Detailed instructions for the exercise
- `type` (string): "simulation", "coding", "analysis", "design", or "experiment"
- `estimatedTime` (number): Time needed in minutes
- `difficultyLevel` (string): "beginner", "intermediate", or "advanced"
- `prerequisites` (string[]): List of skills or knowledge needed
- `instructions` (string): Step-by-step instructions in MDX format
- `expectedOutcome` (string): What students should achieve
- `validationCriteria` (string[]): List of criteria to verify completion
- `resources` (string[]): List of required resources or files

**Validation Rules**:
- `type` must be one of the allowed values
- `difficultyLevel` must be one of the allowed values
- `estimatedTime` must be between 15-120 minutes
- `expectedOutcome` must be specific and measurable

### Assessment
An evaluation component to measure student understanding of the chapter content.

**Fields**:
- `id` (string): Unique identifier within the chapter
- `title` (string): Descriptive title of the assessment
- `type` (string): "quiz", "project", "practical", or "reflection"
- `questions` (Question[]): Array of assessment questions
- `passingScore` (number): Minimum score required to pass (0-100)
- `timeLimit` (number): Time limit in minutes (0 for untimed)
- `feedback` (string): Feedback to provide after completion
- `rubric` (string): Grading criteria for subjective assessments

**Validation Rules**:
- `type` must be one of the allowed values
- `passingScore` must be between 0-100
- `timeLimit` must be >= 0
- `questions` array must contain 1-10 items for quizzes, 1 item for other types

### Question
An individual question within an assessment.

**Fields**:
- `id` (string): Unique identifier within the assessment
- `text` (string): The question text
- `type` (string): "multiple-choice", "true-false", "short-answer", "coding", or "essay"
- `options` (string[]): For multiple-choice questions, possible answers
- `correctAnswer` (string | number | boolean): The correct answer
- `explanation` (string): Explanation of the correct answer
- `points` (number): Points assigned to the question
- `difficultyLevel` (string): "beginner", "intermediate", or "advanced"

**Validation Rules**:
- `type` must be one of the allowed values
- `options` array required for multiple-choice questions
- `points` must be between 1-10
- `difficultyLevel` must be one of the allowed values

### Visualization
An interactive component that visualizes concepts or data for educational purposes.

**Fields**:
- `id` (string): Unique identifier within the chapter
- `title` (string): Descriptive title of the visualization
- `type` (string): "3d-robot", "graph", "diagram", "simulation", or "chart"
- `description` (string): Brief description of what the visualization shows
- `componentName` (string): Name of the React component to render
- `parameters` (object): Configuration parameters for the visualization
- `interactions` (string[]): List of possible user interactions
- `learningGoals` (string[]): Educational objectives the visualization supports
- `accessibilityLabel` (string): Description for screen readers

**Validation Rules**:
- `type` must be one of the allowed values
- `componentName` must reference an existing React component
- `parameters` must match the component's expected props

### CodeExample
A runnable code sample that demonstrates a concept or technique.

**Fields**:
- `id` (string): Unique identifier within the chapter
- `title` (string): Descriptive title of the code example
- `language` (string): Programming language ("python", "cpp", "bash", etc.)
- `code` (string): The actual code content
- `description` (string): Explanation of what the code does
- `expectedOutput` (string): What the code should produce
- `prerequisites` (string[]): List of requirements to run the code
- `setupInstructions` (string): How to set up the environment
- `runInstructions` (string): How to execute the code
- `relatedConcepts` (string[]): Concepts demonstrated by this example

**Validation Rules**:
- `language` must be a supported programming language
- `code` must be syntactically valid
- `expectedOutput` must match the code behavior

### Reference
An academic or technical source cited in the textbook.

**Fields**:
- `id` (string): Unique identifier for the reference
- `type` (string): "academic", "technical-documentation", "standard", or "web-resource"
- `title` (string): Title of the referenced work
- `authors` (string[]): List of authors
- `year` (number): Publication year
- `source` (string): Journal, conference, or publisher
- `url` (string): URL if available (optional)
- `doi` (string): DOI if available (optional)
- `citation` (string): Properly formatted citation in APA/IEEE style
- `accessDate` (string): Date when the resource was accessed

**Validation Rules**:
- `type` must be one of the allowed values
- `year` must be between 1950 and current year
- Either `url` or `doi` must be provided for web resources
- `citation` must follow APA or IEEE format as specified

### HardwareSpecification
Technical requirements for student workstations, edge computing, and robot platforms needed for the curriculum.

**Fields**:
- `id` (string): Unique identifier (e.g., "digital-twin-rig", "edge-computing-kit", "robot-platform")
- `name` (string): Descriptive name
- `category` (string): "workstation", "edge-computing", or "robot-platform"
- `requirements` (object): Specific hardware requirements
- `recommended` (object): Recommended specifications
- `minimum` (object): Minimum viable specifications
- `software` (string[]): Required software components
- `costRange` (string): Estimated cost range
- `alternatives` (string[]): Alternative options
- `setupGuide` (string): Instructions for setup and configuration

**Validation Rules**:
- `category` must be one of the allowed values
- `requirements` must include CPU, RAM, GPU, and storage specifications
- `minimum` specifications must be less than or equal to `recommended`

---

## Entity Relationships

### Chapter Relationships
- A Chapter contains multiple LearningObjectives (1 to many)
- A Chapter contains multiple Exercises (1 to many)
- A Chapter contains multiple Assessments (1 to many)
- A Chapter contains multiple Visualizations (1 to many)
- A Chapter contains multiple CodeExamples (1 to many)
- A Chapter references multiple References (1 to many)

### Assessment Relationships
- An Assessment contains multiple Questions (1 to many)

### Exercise Relationships
- An Exercise may be associated with multiple LearningObjectives (many to many via relatedExercises field)

### LearningObjective Relationships
- A LearningObjective may be verified by multiple Exercises (many to many via relatedExercises field)

---

## State Transitions

### Chapter States
- `draft`: Initial state, content is being created
- `review`: Content is under review
- `approved`: Content has been approved for publication
- `published`: Content is live and accessible
- `deprecated`: Content is outdated but still accessible for reference

### Assessment States
- `draft`: Assessment is being created
- `active`: Assessment is available for students
- `inactive`: Assessment is temporarily unavailable
- `archived`: Assessment is no longer used

### Exercise States
- `draft`: Exercise is being created
- `active`: Exercise is available for students
- `needs-update`: Exercise needs modification based on feedback
- `deprecated`: Exercise is no longer recommended

---

## Data Validation Rules

### Content Validation
- All content must be Docusaurus-compatible MDX
- All code examples must be syntactically valid
- All external links must be verified
- All citations must reference actual sources

### Educational Validation
- Learning objectives must be measurable
- Exercises must have clear expected outcomes
- Assessments must have clear scoring criteria
- Content must progress from simple to complex

### Technical Validation
- All component references must exist
- All file paths must be valid
- All images must have appropriate alt text
- All interactive elements must have accessibility attributes

---

## Indexes and Performance Considerations

### Primary Indexes
- Chapter.id (unique)
- Chapter.moduleNumber (sorted)
- Reference.id (unique)
- Assessment.id (unique)

### Performance Optimizations
- Cache frequently accessed content
- Lazy load visualizations
- Optimize images and assets
- Preload critical resources