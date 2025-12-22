---
id: 6
title: Fix Duplicate Sitemap Plugin Error
stage: red
date: 2025-12-22
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: user
command: /sp.phr
labels: [docusaurus, sitemap, configuration, error-fix, deployment]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - my-robotics-textbook/docusaurus.config.ts
tests:
  - none
---

## Prompt

--title "Fix-Duplicate-Sitemap-Plugin" --stage "red"

## Response snapshot

Fixed the duplicate sitemap plugin configuration error that was preventing Vercel deployment. The issue was that the sitemap plugin was configured in both the plugins array and the classic preset, causing a "Plugin 'docusaurus-plugin-sitemap' is used 2 times with ID 'default'" error. The fix involved:
1. Removing the manual sitemap plugin from the plugins array
2. Moving the sitemap configuration to the classic preset where it belongs
3. Keeping the deprecated onBrokenMarkdownLinks option in its proper place

## Outcome

- ✅ Impact: Resolved duplicate plugin error that was blocking Vercel deployment
- 🧪 Tests: none
- 📁 Files: Modified docusaurus.config.ts to fix plugin configuration
- 🔁 Next prompts: Ready to deploy to Vercel
- 🧠 Reflection: Proper Docusaurus configuration prevents build errors and deployment failures

## Evaluation notes (flywheel)

- Failure modes observed: Duplicate plugin configuration causing build failures
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Deploy to Vercel to confirm fix works