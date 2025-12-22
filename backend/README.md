# RAG Knowledge Ingestion Pipeline

This project implements a RAG (Retrieval-Augmented Generation) knowledge ingestion pipeline that crawls book URLs, extracts content, generates embeddings, and stores them in Qdrant for semantic search.

## Features

- Crawls book URLs from sitemap.xml
- Extracts text content from web pages using trafilatura
- Chunks text into manageable pieces
- Generates embeddings using Cohere
- Stores vectors in Qdrant with metadata

## Setup

1. Install dependencies:
   ```bash
   uv sync
   ```

2. Run the ingestion pipeline:
   ```bash
   uv run main.py
   ```

## Configuration

The pipeline uses the following services:
- **Cohere**: For generating text embeddings
- **Qdrant**: Vector database for storing embeddings
- **Sitemap URL**: `https://physical-ai-humanoid-robotics-gamma-ten.vercel.app/sitemap.xml`

## Architecture

The main components are:
- `get_all_urls()`: Extracts URLs from sitemap
- `extract_text_from_url()`: Extracts text content from a URL
- `chunk_text()`: Splits text into chunks
- `embed()`: Creates embeddings using Cohere
- `create_collection()`: Sets up Qdrant collection
- `save_chunk_to_qdrant()`: Stores embeddings in Qdrant
- `ingest_book()`: Main ingestion pipeline