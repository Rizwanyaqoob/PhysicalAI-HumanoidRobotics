# Data Model: Physical AI & Humanoid Robotics Book

## Content Structure

### Educational Content Entity
```
{
  "id": "unique-content-id",
  "title": "Content title",
  "description": "Brief description",
  "content": "Markdown/MDX content",
  "prerequisites": ["list", "of", "prerequisites"],
  "learningObjectives": ["list", "of", "objectives"],
  "chapterStructure": {
    "motivation": "Section explaining why this topic matters",
    "coreConcepts": "Theoretical foundations",
    "practicalExamples": "Real-world applications",
    "codeBlocks": ["list", "of", "executable", "code", "examples"],
    "troubleshooting": "Common issues and solutions",
    "quiz": "Assessment questions"
  },
  "codeExamples": [
    {
      "language": "python|c++|bash|etc",
      "code": "actual code",
      "explanation": "explanation of the code",
      "environment": "ros2|gazebo|unity|isaac",
      "tested": boolean (verified to work with specified versions)
    }
  ],
  "frameworkRequirements": {
    "ros2": {
      "version": "Humble or newer",
      "packages": ["list", "of", "required", "packages"],
      "dependencies": ["list", "of", "dependencies"]
    },
    "gazebo": {
      "version": "Fortress or newer",
      "plugins": ["list", "of", "required", "plugins"],
      "models": ["list", "of", "required", "models"]
    },
    "unity": {
      "version": "LTS",
      "packages": ["list", "of", "required", "packages"],
      "scenes": ["list", "of", "required", "scenes"]
    },
    "isaac": {
      "version": "Current release",
      "apps": ["list", "of", "required", "apps"],
      "pipelines": ["list", "of", "required", "pipelines"]
    }
  },
  "relatedTopics": ["list", "of", "related", "topics"],
  "difficultyLevel": "beginner|intermediate|advanced|multi-tiered",
  "estimatedTime": "Time to complete chapter"
}
```

### Chapter Navigation Structure
- **Sequential Learning Path**: Chapters arranged in logical progression from fundamentals to advanced topics
- **Cross-References**: Links between related concepts across chapters
- **Prerequisite Tracking**: Dependencies between chapters and topics

### Code Example Entity
```
{
  "id": "unique-code-example-id",
  "title": "Example title",
  "description": "Brief description of what the example demonstrates",
  "language": "programming language",
  "code": "executable code content",
  "environment": "target environment (ros2, gazebo, unity, etc.)",
  "frameworkVersions": {
    "ros2": "specific version requirement",
    "gazebo": "specific version requirement",
    "unity": "specific version requirement",
    "isaac": "specific version requirement"
  },
  "prerequisites": ["system setup", "dependencies", "previous knowledge"],
  "expectedOutput": "Description of what the example should produce",
  "troubleshooting": ["common issues", "solutions"],
  "testedStatus": "verified|needs_verification|broken",
  "lastTested": "date of last verification",
  "relatedExamples": ["list", "of", "related", "examples"]
}
```