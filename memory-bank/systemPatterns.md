# System Patterns

## System Architecture

The system primarily revolves around a Git repository. The architecture will evolve based on the software being developed within this repository. Currently, the focus is on version control itself.

## Key Technical Decisions

- **Version Control:** Git is the chosen version control system.
- **Documentation:** A "Memory Bank" system using Markdown files is in place to maintain project context and history, crucial due to the AI's session-based memory.

## Design Patterns in Use

- **Memory Bank:** A structured documentation approach to persist knowledge across sessions.

## Component Relationships

- The core component is the Git repository.
- The Memory Bank files (`projectbrief.md`, `productContext.md`, `activeContext.md`, `systemPatterns.md`, `techContext.md`, `progress.md`) are critical for the AI's operational context.
