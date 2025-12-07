# ADR-2: Development Language

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 001-docusaurus-docs
- **Context:** The project requires a programming language for configuring the documentation platform, creating custom React components, and writing any necessary build or test scripts. The language choice affects developer experience, code quality, and long-term maintainability.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

TypeScript will be the primary language used for all custom development and configuration files (`.ts`, `.tsx`).

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Enforces type safety, catching potential errors during development rather than at runtime.
- Improves developer experience through better autocompletion, static analysis, and refactoring capabilities in modern IDEs.
- Leads to more maintainable and self-documenting code, which is critical for a long-term project.

### Negative

- Adds a compilation step to the development process.
- Presents a slightly steeper learning curve for developers who are not familiar with static typing concepts.

## Alternatives Considered

- **Pure JavaScript (ES6+)**: This was the primary alternative. It is simpler to get started with and requires no compilation step. However, it was rejected because the benefits of static type checking were deemed critical for ensuring the long-term quality and maintainability of the project, especially when developing custom React components.

## References

- Feature Spec: null
- Implementation Plan: specs/001-docusaurus-docs/plan.md
- Related ADRs: ADR-1
- Evaluator Evidence: specs/001-docusaurus-docs/research.md
