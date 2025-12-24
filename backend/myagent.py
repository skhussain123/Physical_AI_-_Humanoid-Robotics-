from fastapi import FastAPI
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware

from agents import (
    Agent,
    Runner,
    OpenAIChatCompletionsModel,
    AsyncOpenAI,
    set_tracing_disabled,
    function_tool,
)

import os
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
import uvicorn

# ----------------------------------
# Setup
# ----------------------------------
load_dotenv()
set_tracing_disabled(disabled=True)

app = FastAPI(title="Physical AI Tutor API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ----------------------------------
# Gemini (via OpenAI-compatible API)
# ----------------------------------
gemini_api_key = os.getenv("GEMINI_API_KEY", "AIzaSyDvpLEvKoaFjhVRoCrtfaVEM1v6sD0pzbA")

provider = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=provider,
)

# ----------------------------------
# Cohere + Qdrant
# ----------------------------------
cohere_api_key = os.getenv("COHERE_API_KEY", "p7hkZecf2aHRh9xMdSZXn2zuFeoD9FkgWDCg5iwq")
cohere_client = cohere.Client(cohere_api_key)

qdrant_url = os.getenv("QDRANT_URL", "https://c176bce6-b619-4b0e-9795-fc9ad86c1d50.europe-west3-0.gcp.cloud.qdrant.io:6333")
qdrant_api_key = os.getenv("QDRANT_API_KEY", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.N0nFb78VMh4xda6t1IEB41kmrA2AiUD1qWl-6ARXO5w")
qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key
)


def get_embedding(text: str):
    """Get embedding from Cohere"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",
        texts=[text],
    )
    return response.embeddings[0]


@function_tool
def retrieve(query: str):
    """Retrieve relevant textbook chunks from Qdrant"""
    embedding = get_embedding(query)

    result = qdrant.search(
        collection_name="humanoid_ai_book",
        query_vector=embedding,
        limit=5,
    )

    return [point.payload["text"] for point in result]


# ----------------------------------
# Agent
# ----------------------------------
agent = Agent(
    name="Physical-AI-Tutor",
    instructions="""
You are an AI tutor for the Physical AI & Humanoid Robotics textbook.

Steps:
1. Always call the `retrieve` tool with the user query
2. Answer ONLY from retrieved content
""",
    model=model,
    tools=[retrieve],
)

# ----------------------------------
# API Schema
# ----------------------------------
class ChatRequest(BaseModel):
    question: str


class ChatResponse(BaseModel):
    answer: str


# ----------------------------------
# API Endpoint
# ----------------------------------
@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    result = await Runner.run(
        agent,
        input=request.question,
    )

    return ChatResponse(answer=result.final_output)


# ----------------------------------
# Run Server
# ----------------------------------
if __name__ == "__main__":
    uvicorn.run(
        "myagent:app",
        host="127.0.0.3",
        port=8001,
        reload=True,
    )
