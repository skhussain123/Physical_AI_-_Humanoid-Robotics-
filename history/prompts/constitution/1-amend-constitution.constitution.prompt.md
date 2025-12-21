---
id: 1
title: Amend Constitution
stage: constitution
date: 2025-12-19
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: user
command: /sp.constitution
labels: [constitution, specification, governance]
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

AI/Spec-Driven Book with Integrated RAG Chatbot

Core principles:
- Specification-driven development using Spec-Kit Plus
- Accuracy and technical correctness across all book content
- Clear, accessible writing suitable for developers and students
- Maintainability of code examples, architecture diagrams, and configuration files
- Consistency across chapters, components, and generated assets
- Reproducible builds using Claude Code and Docusaurus workflows

Key standards:
- All technical explanations must be validated against official documentation
  (Docusaurus, OpenAI Agents/ChatKit SDK, FastAPI, Qdrant, Neon Postgres, GitHub Pages)
- Code samples must be runnable, tested, and version-compatible
- Architecture explanations must include diagrams, workflows, and reasoning
- Writing tone: simple, teacher-like, technically precise
- All features, components, and chapters must be spec-driven using Spec-Kit Plus
- GitHub Pages deployment instructions must be accurate and reproducible

Book requirements:
- Written using Docusaurus
- Automatically buildable/deployable to GitHub Pages
- Includes step-by-step tutorials, code samples, and diagrams
- Includes a fully embedded RAG chatbot that:
  - Uses OpenAI Agents/ChatKit SDK
  - Uses FastAPI backend
  - Uses Qdrant (Free Tier) vector database
  - Uses Neon Serverless Postgres
  - Can answer questions about book content
  - Can answer questions based only on selected text by the user

RAG chatbot standards:
- Retrieval pipeline must be fully documented
- Chunking strategy must be described and configurable
- Qdrant schema must be validated and version-controlled
- FastAPI implementation must follow best practices
- Model responses must cite retrieved text
- UI/UX must be embedded directly in the Docusaurus UI

Constraints:
- Consistent chapter structure across the entire book
- All diagrams generated using Claude Code or external plugins
- No broken links, code errors, or missing configuration files
- All text output must follow the constitution rules
- Deployment is mandatory and must be automated

Success criteria:
- Book successfully builds and deploys on GitHub Pages
- RAG chatbot fully operational inside the published Docusaurus site
- All content generated via Spec-Kit Plus matches the constitution
- All instructions can be followed by students with minimal friction
- All components (specs, skills, pages, backend, chatbot) are unified and consistent
- The project demonstrates complete Spec-Driven Development workflow

## Response snapshot

Updated the project constitution file (.specify/memory/constitution.md) to reflect the specific requirements for the AI/Spec-Driven Book with Integrated RAG Chatbot project. The constitution now includes six core principles, additional constraints, and success criteria that align with the project goals. The file includes a Sync Impact Report at the top, and all placeholders from the original template have been filled with concrete values based on the user input.

## Outcome

- ✅ Impact: Updated project constitution to align with AI/Spec-Driven Book requirements
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with new principles and governance
- 🔁 Next prompts: Plan and implement RAG chatbot components
- 🧠 Reflection: Constitution now clearly defines the project's core principles and constraints

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Begin feature specification for RAG chatbot