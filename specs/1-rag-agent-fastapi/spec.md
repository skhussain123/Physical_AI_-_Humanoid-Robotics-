# RAG Agent Backend with FastAPI Specification

## Feature Overview

**Feature**: RAG Agent Backend with FastAPI Integration for AI Tutoring
**Short Name**: rag-agent-fastapi
**Target Audience**: Developers and users of the Physical AI & Humanoid Robotics textbook AI tutor
**Focus**: Expose an AI tutoring agent as a FastAPI backend using OpenAI Agents SDK and Qdrant embeddings

## User Scenarios & Testing

### Primary User Scenario
As a user of the Physical AI & Humanoid Robotics textbook AI tutor, I want to interact with the AI tutor through a web API so that I can ask questions about the textbook content and receive answers based on the retrieved knowledge.

### Secondary User Scenario
As a developer integrating the AI tutor into other applications, I want to make HTTP requests to a FastAPI backend so that I can programmatically query the AI tutor and receive responses.

### User Flow
1. User sends a query to the /ask endpoint via HTTP POST request
2. FastAPI receives the query and passes it to the OpenAI Agent
3. The agent uses the `retrieve` tool to fetch relevant content from Qdrant
4. The agent generates a response using only the retrieved content
5. If no relevant content is found, the agent responds with "I don't know"
6. FastAPI returns the response to the user
7. User can also check the service status via the /check endpoint

### Acceptance Criteria
- [ ] FastAPI application successfully initializes with OpenAI Agents SDK
- [ ] POST /ask endpoint accepts user queries and returns AI-generated responses
- [ ] GET /check endpoint returns a hello world message confirming service is running
- [ ] Agent uses Cohere embeddings to query Qdrant collection for relevant content
- [ ] Agent answers questions using ONLY the retrieved Qdrant content
- [ ] Agent responds with "I don't know" when no relevant content is found
- [ ] Backend runs locally via uvicorn without errors
- [ ] All logic is contained in a single backend file (backend/myagent.py)
- [ ] API endpoints are properly documented with FastAPI's automatic documentation

## Functional Requirements

### FR-1: FastAPI Application Setup
- The system SHALL initialize a FastAPI application instance
- The system SHALL expose API endpoints according to the specified routes
- The system SHALL provide automatic API documentation via FastAPI's built-in features
- The system SHALL handle request/response serialization automatically

### FR-2: OpenAI Agents SDK Integration
- The system SHALL initialize the OpenAI Agent with proper configuration
- The system SHALL configure the agent to use the specified model (gemini-2.5-flash)
- The system SHALL register the `retrieve` tool for the agent to use during conversations
- The system SHALL configure the agent with proper instructions for using retrieved content

### FR-3: Cohere Embedding Generation
- The system SHALL initialize a Cohere client with the provided API key
- The system SHALL generate embeddings using Cohere Embed v3 model
- The system SHALL use "search_query" input type for user queries when generating embeddings
- The system SHALL handle embedding generation errors gracefully

### FR-4: Qdrant Retrieval Integration
- The system SHALL connect to the Qdrant Cloud instance with the provided URL and API key
- The system SHALL query the "humanoid_ai_book" collection for relevant content
- The system SHALL limit query results to 5 most relevant documents
- The system SHALL extract text content from retrieved Qdrant points

### FR-5: /ask Endpoint Implementation
- The system SHALL expose a POST endpoint at /ask to accept user queries
- The system SHALL accept JSON requests with a "query" field containing the user's question
- The system SHALL pass the user query to the OpenAI Agent for processing
- The system SHALL return the agent's response in JSON format
- The system SHALL handle errors gracefully and return appropriate HTTP status codes

### FR-6: /check Endpoint Implementation
- The system SHALL expose a GET endpoint at /check for service health checking
- The system SHALL return a simple "hello world" message or status confirmation
- The system SHALL be accessible without authentication

### FR-7: Content Retrieval Logic
- The system SHALL ensure the agent ONLY uses content retrieved from Qdrant to answer questions
- The system SHALL implement the `retrieve` tool function that queries Qdrant with embeddings
- The system SHALL configure the agent with instructions to respond with "I don't know" when no relevant content is found
- The system SHALL preserve the agent's tutoring persona for the Physical AI & Humanoid Robotics textbook

## Non-Functional Requirements

### Performance Requirements
- The system SHALL respond to queries within reasonable timeframes (under 10 seconds for typical questions)
- The system SHALL handle concurrent requests appropriately
- The system SHALL efficiently use API resources from Cohere and Qdrant

### Security Requirements
- The system SHALL securely handle API keys and credentials through environment variables or secure storage
- The system SHALL validate input to prevent injection attacks
- The system SHALL implement appropriate rate limiting if needed

### Reliability Requirements
- The system SHALL handle network failures with Qdrant and Cohere gracefully
- The system SHALL provide appropriate error messages when external services are unavailable
- The system SHALL maintain consistent behavior when no relevant content is found

### Scalability Requirements
- The system SHALL be designed to handle multiple concurrent users
- The system SHALL efficiently manage resources during high load periods

## Success Criteria

- Successfully initialize FastAPI application with OpenAI Agents SDK with 100% success rate
- Expose POST /ask endpoint that accepts user queries and returns AI-generated responses with 99% success rate
- Expose GET /check endpoint that returns hello world message confirming service status with 100% success rate
- Agent correctly uses Cohere embeddings to retrieve content from Qdrant collection with 99% success rate
- Agent answers questions using ONLY retrieved Qdrant content with 100% adherence to constraint
- Agent responds with "I don't know" when no relevant content is found with 100% accuracy
- Backend runs locally via uvicorn without errors during 24-hour stability test
- All logic successfully contained in a single backend file (backend/myagent.py) with proper organization
- API endpoints properly documented with FastAPI's automatic documentation accessible at /docs

## Key Entities

### User Query
- **Attributes**: query (string)
- **Description**: The question or input from the user to be processed by the AI tutor

### AI Response
- **Attributes**: response (string)
- **Description**: The answer generated by the AI agent based on retrieved content

### Agent Configuration
- **Attributes**: model_name (string), api_key (string), base_url (string), tools (array of functions)
- **Description**: Configuration parameters for the OpenAI Agent including model and tools

### Qdrant Connection
- **Attributes**: url (string), api_key (string), collection_name (string)
- **Description**: Connection parameters for accessing the Qdrant vector database

### Cohere Client
- **Attributes**: api_key (string), model_name (string), input_type (string)
- **Description**: Configuration for generating embeddings with Cohere's embedding service

### API Endpoint
- **Attributes**: method (string), path (string), request_schema (object), response_schema (object)
- **Description**: Definition of the FastAPI endpoints with their HTTP methods and data schemas

## Assumptions

- The Qdrant collection "humanoid_ai_book" exists and contains properly indexed textbook content
- Cohere API is available and responsive during query processing
- Qdrant Cloud is available and responsive during query processing
- The OpenAI Agents SDK is compatible with the configured model (gemini-2.5-flash)
- The user has valid API credentials for Cohere, Qdrant Cloud, and the model provider
- The textbook content in Qdrant is properly formatted and contains relevant information
- The system has network access to external APIs (Cohere, Qdrant, model provider)

## Constraints

- Language: Python
- Framework: FastAPI
- Agent SDK: OpenAI Agents
- Embeddings source: Cohere (embed-english-v3.0 model)
- Vector DB: Qdrant Cloud (existing collection)
- All logic must remain in a single file: backend/myagent.py
- Agent must ONLY use retrieved content to answer questions
- Agent must respond with "I don't know" when no relevant content is found
- Must support local execution via uvicorn

## Dependencies

- FastAPI for web framework
- OpenAI Agents SDK for AI agent functionality
- Cohere API for embedding generation
- Qdrant Client for vector database access
- uvicorn for ASGI server
- Python 3.x runtime environment