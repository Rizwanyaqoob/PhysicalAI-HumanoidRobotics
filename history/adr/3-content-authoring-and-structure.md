# ADR-3: Content Authoring and Structure

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 001-docusaurus-docs
- **Context:** A clear and maintainable content strategy is needed for a large documentation project. This includes how content is organized on the file system, how it is enriched with non-textual elements like code and diagrams, and how the overall navigation is managed.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

The content authoring strategy is a cluster of three related decisions:
- **Organization**: Content will be organized into thematic, modular sections corresponding to subdirectories within the `docs/` folder. Navigation and structure will be managed centrally via `sidebars.js`.
- **Code Examples**: Code will be integrated directly into Markdown/MDX files using standard code blocks with syntax highlighting.
- **Diagrams**: Diagrams will be created using Mermaid syntax directly within Markdown files, allowing them to be version-controlled as text.

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- The modular directory structure is intuitive, mirrors the book's structure, and is easily scalable.
- Managing sidebars in a central file (`sidebars.js`) provides clear and explicit control over the site's navigation hierarchy.
- Embedding code and diagrams directly with the relevant content simplifies the authoring workflow and keeps related items colocated.
- Version-controlling diagrams as text (Mermaid) is a significant advantage over managing binary image files, as it simplifies diffing and collaboration.

### Negative

- Very complex diagrams may be difficult to create and maintain using Mermaid syntax, potentially requiring a fallback to static images in rare cases.
- Large code blocks embedded directly inside Markdown files can sometimes make the source content harder to read and navigate.

## Alternatives Considered

- **Organization**: 
  - A single-page documentation site was rejected as it is not scalable for a large volume of content and is difficult to navigate.
  - A pure blog format was rejected as it is not suitable for a structured, chapter-based learning path.
- **Code Examples**: 
  - Linking to external repositories or gists was rejected due to the maintenance overhead of keeping links and code in sync.
- **Diagrams**: 
  - Using static image files (e.g., PNG, SVG) was rejected because they are harder to version control, review, and update.
  - Relying on external diagramming tools was rejected as it adds external dependencies to the authoring workflow.

## References

- Feature Spec: null
- Implementation Plan: specs/001-docusaurus-docs/plan.md
- Related ADRs: ADR-1
- Evaluator Evidence: specs/001-docusaurus-docs/research.md
