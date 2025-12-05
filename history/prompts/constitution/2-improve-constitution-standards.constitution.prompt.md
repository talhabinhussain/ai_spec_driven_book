---
id: 2
title: Improve constitution standards
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.phr
labels: [constitution, improvement, standards]
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

Review my Constitution at .specify/memory/constitution.md and improve it:

1. Are all standards testable (not vague)?
   - ❌ Vague: "Papers should be well-written"
   - ✅ Testable: "Flesch-Kincaid grade 10-12; active voice 75%+ of time"

2. Did I cover essential categories?
   - Citation accuracy
   - Source verification
   - Writing clarity
   - Plagiarism checking
   - Review process

3. Are any standards unrealistic?

Suggest 2-3 concrete improvements.

## Response snapshot

Improved the constitution by making the Clarity principle more testable, adding plagiarism checking, and relaxing the source use requirements.

## Outcome

- ✅ Impact: Constitution improved with more testable standards and better coverage.
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: Review the updated constitution.
- 🧠 Reflection: It's important to balance aspirational goals with realistic and testable standards.

## Evaluation notes (flywheel)

- Failure modes observed: Could not execute shell script; using agent-native tools instead.
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Fix the shell script execution.
