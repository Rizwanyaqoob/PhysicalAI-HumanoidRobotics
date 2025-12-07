# ADR-1: Documentation Platform Stack

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 001-docusaurus-docs
- **Context:** The project requires a robust, feature-rich platform to create and maintain a comprehensive documentation site for "Humanoid Robotics: A Practical Introduction". The platform must support technical documentation features like code examples, diagrams, search, and versioning, and be compatible with a static hosting environment like GitHub Pages.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

The chosen documentation platform stack consists of:
- **Platform**: Docusaurus v3
- **Core Framework**: React
- **Content Format**: MDX (Markdown with JSX)
- **Bundler**: Webpack (comes with Docusaurus)

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Excellent out-of-the-box support for technical documentation needs (search, versioning, theming).
- Integrated and cohesive ecosystem (React, MDX, Webpack) managed by the Docusaurus platform.
- Strong community support and good performance for static sites.
- Enables creation and embedding of custom React components directly within Markdown content.

### Negative

- Couples the content and presentation layer to a React-based framework, making future migration to a non-React platform more difficult.
- Can be more complex than simpler static site generators if advanced features are not required.

## Alternatives Considered

- **GitBook**: Offers a good user interface and experience but is less flexible for custom components and theming compared to Docusaurus.
- **MkDocs**: Simpler to set up and great for basic documentation, but lacks the rich feature set of Docusaurus, particularly the ability to embed interactive React components.
- **Sphinx**: A powerful tool, especially in the Python ecosystem, but less suited for a mixed-content project where JavaScript/React components are desired.

## References

- Feature Spec: null
- Implementation Plan: specs/001-docusaurus-docs/plan.md
- Related ADRs: null
- Evaluator Evidence: specs/001-docusaurus-docs/research.md
