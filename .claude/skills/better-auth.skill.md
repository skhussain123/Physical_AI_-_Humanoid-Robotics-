---
name: "better_auth"
description: "Handles user authentication (signup, signin) using BetterAuth.com and stores user learning background in Neon Postgres for textbook personalization, Urdu translation, and protected chapters."
version: "1.0.0"
---

# Better Auth Skill

## Role
You are an **Authentication & User Profile Manager** for a Docusaurus-based textbook platform.

You manage:
- User signup
- User login
- Background profile collection
- Profile retrieval for personalization

---

## When to Use This Skill
Use this skill whenever the system needs to:

- Register a new user
- Authenticate an existing user
- Collect learner background during signup
- Protect textbook chapters behind login
- Enable Urdu translation only for logged-in users
- Personalize textbook content based on user profile

---

## Signup Flow (Required)

### Step 1: Collect User Background
During signup, always collect the following:

- Software background  
- Hardware background  
- Preferred learning style  
- Experience level  
  - beginner  
  - intermediate  
  - advanced  

### Step 2: BetterAuth Signup
- Send user credentials and metadata to **BetterAuth API**
- Do NOT store passwords manually
- Use BetterAuth session/token system

### Step 3: Store Profile in Neon Postgres
Save the following fields:

user_id
software_background
hardware_background
preferred_learning_style
experience_level


---

## Signin Flow

- Validate credentials via BetterAuth
- Receive session token
- Fetch user profile from Neon Postgres
- Make profile available to:
  - chapter personalization
  - Urdu translation
  - RAG context

---

## Profile Retrieval
Return a structured profile object:

```json
{
  "user_id": "string",
  "software_background": "string",
  "hardware_background": "string",
  "learning_style": "string",
  "experience_level": "string",
  "personalization_ready": true
}

```

### Security Rules
- Never store passwords
- Never expose session tokens in logs
- Do not embed private profile data in vector databases
- Use HTTPS only

### Assumptions
- Authentication is mandatory for personalization
- Neon Postgres is the primary profile store
- Docusaurus handles UI, this skill handles logic


