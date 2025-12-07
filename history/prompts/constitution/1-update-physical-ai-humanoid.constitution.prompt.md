---
id: 1
title: Update Physical AI Humanoid Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution,physical-ai,robotics,setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

/sp.constitution

Purpose
Define how the system should create and maintain the "Physical AI & Humanoid Robotics" book. The goal is predictable structure, consistent writing, and clean automation.

Authoring Rules

Write like a technical instructor who knows robotics.

Keep sentences short and clear.

Prioritize accuracy over hype.

Explain concepts in plain English with correct terminology.

Show real code, not pseudocode, unless the concept requires abstraction.

Use examples that can run in ROS 2 Humble or newer, Gazebo Fortress or newer, Unity LTS, and Isaac Sim current release.

Structure Rules

Every chapter must have:

A motivation section

Core concepts

Practical examples

Code blocks

Troubleshooting

A short quiz

All code must be runnable or clearly marked as illustrative.

Follow Docusaurus formatting (MDX or Markdown only).

Versioning

Use semantic versioning for book updates.

All major content changes require a new version folder under versioned_docs/.

Contribution Workflow

Every generat

## Response snapshot

Updated the project constitution at .specify/memory/constitution.md with the following changes:

1. Set project name to "Physical AI & Humanoid Robotics"
2. Added 6 core principles:
   - Technical Accuracy First
   - Practical Examples
   - Structured Chapter Format
   - Docusaurus Compliance
   - Semantic Versioning
   - Clear Writing Style
3. Added Technology Stack Requirements section
4. Added Contribution Workflow section
5. Added Governance section
6. Set version to 1.0.0, ratified and last amended to 2025-12-06

## Outcome

- ✅ Impact: Successfully created project constitution with all required principles and guidelines
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with new constitution content
- 🔁 Next prompts: Ready for feature specification and planning
- 🧠 Reflection: The constitution now properly reflects the requirements for the Physical AI & Humanoid Robotics book project

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification based on the new constitution