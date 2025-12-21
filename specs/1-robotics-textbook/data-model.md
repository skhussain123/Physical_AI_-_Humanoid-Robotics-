# Data Model: Physical AI & Humanoid Robotics Textbook

## Core Entities

### Student
- **studentId**: string (unique identifier)
- **email**: string (user login credential)
- **name**: string (display name)
- **enrollmentDate**: Date (when user started using the textbook)
- **progress**: Map<string, ModuleProgress> (tracking progress across modules)
- **completedCertificates**: Array<Certificate> (earned certificates)
- **preferences**: UserPreferences (display and accessibility settings)

**Validation rules**:
- email must be valid email format
- name must be 1-100 characters
- studentId must be unique

### Module
- **moduleId**: string (unique identifier like "module1-ros2")
- **title**: string (display title like "The Robotic Nervous System (ROS 2)")
- **description**: string (brief description of the module)
- **duration**: string (estimated completion time)
- **chapters**: Array<Chapter> (list of chapters in the module)
- **learningObjectives**: Array<string> (what students will learn)
- **prerequisites**: Array<string> (required knowledge)

### Chapter
- **chapterId**: string (unique identifier like "module1-chapter1")
- **title**: string (chapter title)
- **content**: string (the actual content in Markdown format)
- **codeExamples**: Array<CodeExample> (code examples in this chapter)
- **learningOutcomes**: Array<string> (what students will achieve)
- **moduleRef**: string (reference to parent module)

### CodeExample
- **exampleId**: string (unique identifier)
- **title**: string (description of the example)
- **code**: string (the actual code content)
- **language**: string (programming language identifier)
- **environment**: string (execution environment type)
- **expectedOutput**: string (what the example should produce)
- **instructions**: string (how to use/run the example)

### Progress
- **studentId**: string (reference to student)
- **moduleId**: string (reference to module)
- **chapterProgress**: Map<string, boolean> (completion status of each chapter)
- **overallCompletion**: number (percentage complete)
- **lastAccessed**: Date (when student last accessed)
- **timeSpent**: number (time spent in seconds)

### Certificate
- **certificateId**: string (unique identifier)
- **studentId**: string (recipient)
- **moduleId**: string (module completed)
- **issueDate**: Date (when issued)
- **title**: string (certificate title)
- **content**: string (description of achievement)

### UserPreferences
- **darkMode**: boolean (whether to use dark theme)
- **fontSize**: string (text size preference)
- **highContrast**: boolean (accessibility setting)
- **language**: string (preferred language)
- **codeTheme**: string (code block syntax highlighting theme)

## Relationships

```
Student 1 -----> * Progress
Student 1 -----> * Certificate
Module 1 -----> * Chapter
Chapter 1 -----> * CodeExample
Student 1 -----> 1 UserPreferences
```

## State Transitions

### Student Progress
- **Not Started** → **In Progress**: When student begins first chapter
- **In Progress** → **Completed**: When student completes all chapters in module
- **Completed** → **Certified**: When certificate is issued

### Chapter Status
- **Locked** → **Unlocked**: When prerequisites are met
- **Unlocked** → **Started**: When student begins chapter
- **Started** → **Completed**: When student marks as complete or completes activities

## Validation Rules

1. A student cannot access a module without meeting prerequisites
2. Progress tracking must be consistent across devices for registered users
3. Certificates can only be issued for completed modules
4. Code examples must be validated before execution
5. User preferences must be persisted across sessions