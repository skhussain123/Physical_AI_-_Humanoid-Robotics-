# Quickstart Guide: RAG Agent Backend with FastAPI

## Overview
This guide will help you quickly set up and run the RAG Agent Backend for the Physical AI & Humanoid Robotics textbook tutor.

## Prerequisites
- Python 3.11 or higher
- pip package manager
- Access to Cohere API
- Access to Qdrant Cloud
- Access to the model provider API (Google via OpenAI-compatible API)

## Installation

1. **Clone or navigate to the project directory:**
   ```bash
   cd C:\Users\user\Music\textbook-hackathon-app
   ```

2. **Install dependencies:**
   ```bash
   cd backend
   pip install fastapi uvicorn python-dotenv
   # Install other existing dependencies as needed
   ```

3. **Set up environment variables:**
   Create a `.env` file in the backend directory with your API keys:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_API_KEY=your_qdrant_api_key
   MODEL_API_KEY=your_model_api_key
   QDRANT_URL=your_qdrant_url
   ```

## Running the Application

1. **Start the server:**
   ```bash
   cd backend
   uvicorn myagent:app --reload --port 8000
   ```

2. **Verify the service is running:**
   Open your browser or use curl to access:
   ```bash
   curl http://localhost:8000/check
   ```

## Using the API

### Health Check
Verify the service is running:
```bash
curl http://localhost:8000/check
```

Expected response:
```json
{
  "message": "Hello World",
  "status": "healthy"
}
```

### Ask a Question
Submit a query to the AI tutor:
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is physical AI?"
  }'
```

Expected response:
```json
{
  "response": "Physical AI refers to the integration of artificial intelligence with physical systems...",
  "status": "success",
  "metadata": {
    "source_documents": ["Chapter 1: Introduction to Physical AI"],
    "confidence": 0.85
  }
}
```

## API Documentation
Access the automatic API documentation at:
- http://localhost:8000/docs (Swagger UI)
- http://localhost:8000/redoc (ReDoc)

## Troubleshooting

### Common Issues

1. **API Keys Not Working**
   - Verify all API keys are correctly set in environment variables
   - Check that the API keys have the necessary permissions

2. **Qdrant Connection Issues**
   - Verify the Qdrant URL and API key are correct
   - Check that the "humanoid_ai_book" collection exists

3. **Model Provider Issues**
   - Verify the model API key and base URL are correct
   - Check that the gemini-2.5-flash model is available

### Error Responses
- 400: Bad Request - Check your request format
- 422: Validation Error - Check your request parameters
- 500: Internal Server Error - Check server logs
- 503: Service Unavailable - Check external service availability

## Environment Variables
- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_API_KEY`: Your Qdrant Cloud API key
- `QDRANT_URL`: Your Qdrant Cloud instance URL
- `MODEL_API_KEY`: API key for the model provider
- `MODEL_BASE_URL`: Base URL for the model provider API (default: Google's OpenAI-compatible API)