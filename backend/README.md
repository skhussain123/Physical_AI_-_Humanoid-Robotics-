# RAG Agent Backend with FastAPI

This is a FastAPI backend implementation for the Physical AI & Humanoid Robotics textbook AI tutor. It exposes an AI tutoring agent using OpenAI Agents SDK, Cohere embeddings, and Qdrant retrieval.

## Features

- FastAPI web application with automatic OpenAPI documentation
- AI tutoring agent that answers questions based on textbook content
- RAG (Retrieval-Augmented Generation) functionality
- Health check endpoint
- Query endpoint with validation and error handling

## API Endpoints

### GET /check
Health check endpoint that returns a simple health status message to confirm the service is running.

Response:
```json
{
  "message": "Hello World",
  "status": "healthy"
}
```

### POST /ask
Submit a question about the Physical AI & Humanoid Robotics textbook content and receive an AI-generated response based on retrieved knowledge.

Request body:
```json
{
  "query": "Your question here",
  "timeout": 30
}
```

Response:
```json
{
  "response": "The AI-generated response",
  "status": "success",
  "metadata": {
    "source_documents": []
  }
}
```

## Configuration

Create a `.env` file in the backend directory with your API keys:

```env
MODEL_API_KEY=your_model_api_key_here
MODEL_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
```

## Installation

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set up environment variables as described above

## Usage

Run the application with uvicorn:

```bash
uvicorn myagent:app --reload --port 8000
```

The API will be available at `http://localhost:8000` with documentation at `http://localhost:8000/docs`

## Deployment

To run in production:

```bash
uvicorn myagent:app --host 0.0.0.0 --port 8000
```

The application follows the constraint that all logic remains in a single file (`myagent.py`) as specified in the requirements.