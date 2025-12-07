    # [PROJECT_NAME] Constitution
    <!-- Sync Impact Report:
    Version change: N/A -> 1.0.0
    Added sections: All principles and sections based on user requirements
    Removed sections: None
    Templates requiring updates:
    - .specify/templates/plan-template.md ✅ updated
    - .specify/templates/spec-template.md ✅ updated
    - .specify/templates/tasks-template.md ✅ updated
    Follow-up TODOs: None
    -->

    # Physical AI & Humanoid Robotics Constitution

    ## Core Principles

    ### Technical Accuracy First
    All content must prioritize technical accuracy over marketing hype; Code examples must be tested and verified to work in ROS 2 Humble or newer, Gazebo Fortress or newer, Unity LTS, and Isaac Sim current release; All explanations use correct terminology with plain English

    ### Practical Examples
    Every concept must include runnable code examples; Code must be real, not pseudocode unless abstraction is required; Examples must work in specified environments (ROS 2, Gazebo, Unity, Isaac Sim)

    ### Structured Chapter Format
    Every chapter must follow the required structure: motivation section, core concepts, practical examples, code blocks, troubleshooting, and short quiz; All code must be runnable or clearly marked as illustrative

    ### Docusaurus Compliance
    All content must follow Docusaurus formatting (MDX or Markdown only); Content must be compatible with versioned_docs/ structure for semantic versioning

    ### Semantic Versioning
    Use semantic versioning for book updates; All major content changes require a new version folder under versioned_docs/

    ### Clear Writing Style
    Write like a technical instructor who knows robotics; Keep sentences short and clear; Explain concepts in plain English with correct terminology

    ## Technology Stack Requirements
    All examples must work with ROS 2 Humble or newer, Gazebo Fortress or newer, Unity LTS, and Isaac Sim current release; Code must be compatible with these environments and clearly specify version requirements

    ## Contribution Workflow
    Every contribution must follow the structured format with all required chapter sections; All code examples must be tested before submission; Content must adhere to Docusaurus MDX/Markdown formatting standards

    ## Governance
    This constitution governs all content creation for the Physical AI & Humanoid Robotics book; All contributions must comply with these principles; Changes to this constitution require explicit approval and documentation

## Software Development & TypeScript Principles

### General Principles
- **Readability**: Code should be written for human understanding first. Use clear variable names and logical structure.
- **Modularity**: Create small, reusable React components and utility functions where applicable.
- **DRY (Don't Repeat Yourself)**: Avoid duplicating code. Abstract common functionality into shared helpers or components.

### TypeScript Best Practices
- **Strict Mode**: Always enable `strict` mode in `tsconfig.json` to leverage the full power of TypeScript's type checking.
- **Explicit Types**: Avoid the `any` type. Be explicit about types for function parameters, return values, and complex objects.
- **Interfaces for Public APIs**: Prefer `interface` for defining the shape of public-facing objects and component props, as they are more easily extended. Use `type` for private or utility types.
- **Readonly Properties**: Use the `readonly` keyword for props and state that should not be mutated to enforce immutability.
- **ESLint Integration**: Use ESLint with TypeScript plugins (`@typescript-eslint/eslint-plugin`) to enforce consistent coding style and catch common errors.

    **Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
