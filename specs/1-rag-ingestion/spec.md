# RAG Knowledge Ingestion Pipeline Specification

## Feature Overview

**Feature**: RAG Knowledge Ingestion Pipeline for Book Website
**Short Name**: rag-ingestion
**Target Audience**: Developers building a RAG-based chatbot over a Docusaurus book
**Focus**: URL ingestion, content extraction, embedding generation, and vector storage

## User Scenarios & Testing

### Primary User Scenario
As a developer building a RAG-based chatbot over a Docusaurus book, I want to automatically ingest content from deployed book URLs so that I can create a knowledge base for my chatbot to answer questions about the book content.

### User Flow
1. Developer configures the ingestion pipeline with website URLs and API credentials
2. Pipeline crawls and collects all deployed book URLs from GitHub Pages
3. Content is extracted, cleaned, and chunked with consistent metadata
4. Embeddings are generated using Cohere embedding models
5. Vectors are stored in Qdrant Cloud with proper indexing
6. Each vector includes source URL, section title, and chunk ID
7. Pipeline supports incremental re-ingestion without duplication

### Acceptance Criteria
- [ ] Pipeline successfully crawls and collects all deployed book URLs
- [ ] Content is properly extracted and cleaned without losing important information
- [ ] Content is chunked with consistent metadata structure
- [ ] Embeddings are generated using Cohere models without errors
- [ ] Vectors are stored in Qdrant Cloud with proper indexing
- [ ] Each vector includes source URL, section title, and chunk ID
- [ ] Pipeline can be executed via CLI or script
- [ ] Pipeline supports incremental re-ingestion without duplication
- [ ] Configuration is handled via environment variables

## Functional Requirements

### FR-1: URL Crawling and Collection
- The system SHALL crawl and collect all deployed book URLs from GitHub Pages
- The system SHALL support crawling of public website URLs only
- The system SHALL handle different URL structures and navigation patterns
- The system SHALL detect and avoid duplicate URLs during crawling

### FR-2: Content Extraction and Cleaning
- The system SHALL extract text content from HTML pages while preserving important structure
- The system SHALL clean extracted content by removing navigation, headers, footers, and other non-content elements
- The system SHALL preserve semantic meaning and context during cleaning
- The system SHALL handle various HTML structures and content formats

### FR-3: Content Chunking
- The system SHALL chunk extracted content into manageable pieces with consistent metadata
- The system SHALL preserve document hierarchy and context within chunks
- The system SHALL ensure chunks are of appropriate size for embedding generation
- The system SHALL maintain relationships between chunks and their source documents

### FR-4: Embedding Generation
- The system SHALL generate embeddings using Cohere embedding models
- The system SHALL use the latest stable Cohere embedding model
- The system SHALL handle API rate limits and errors gracefully
- The system SHALL preserve metadata during embedding generation

### FR-5: Vector Storage
- The system SHALL store vectors in Qdrant Cloud (free tier)
- The system SHALL include source URL, section title, and chunk ID with each vector
- The system SHALL create proper indexing for efficient retrieval
- The system SHALL handle storage errors and connection issues gracefully

### FR-6: Incremental Processing
- The system SHALL support incremental re-ingestion without creating duplicate entries
- The system SHALL identify and skip content that has not changed since last ingestion
- The system SHALL update existing entries when content has changed
- The system SHALL remove entries for content that no longer exists

### FR-7: Configuration and Execution
- The system SHALL be configurable via environment variables
- The system SHALL support CLI-based execution
- The system SHALL support script-based execution
- The system SHALL provide clear error messages and logging

## Non-Functional Requirements

### Performance Requirements
- The system SHALL process content within reasonable timeframes for typical book sizes
- The system SHALL handle rate limiting from Cohere API appropriately
- The system SHALL efficiently use Qdrant Cloud resources within free tier limits

### Security Requirements
- The system SHALL securely handle API keys and credentials through environment variables
- The system SHALL only access public website URLs
- The system SHALL not expose sensitive configuration data

### Reliability Requirements
- The system SHALL handle network failures gracefully
- The system SHALL retry failed operations with appropriate backoff
- The system SHALL maintain data consistency during partial failures

## Success Criteria

- Successfully crawl and collect all deployed book URLs (GitHub Pages) with 95% success rate
- Clean and chunk extracted content with consistent metadata preserving 99% of meaningful content
- Generate embeddings using Cohere embedding models with 99% success rate
- Store vectors in Qdrant Cloud Free Tier with proper indexing and 99% success rate
- Each vector includes source URL, section title, and chunk ID without missing metadata
- Pipeline is reproducible and configurable via environment variables with clear documentation
- Support incremental re-ingestion without duplication with 99% accuracy
- Pipeline execution completes within acceptable timeframes (under 1 hour for typical book size)

## Key Entities

### Book Content Document
- **Attributes**: source_url (string), section_title (string), chunk_id (string), content (text), metadata (object)
- **Description**: Represents a chunk of book content with associated metadata

### Vector Record
- **Attributes**: vector_id (string), embedding (array of floats), payload (object with source_url, section_title, chunk_id)
- **Description**: Represents a vector in the Qdrant database with associated metadata

### Crawl Configuration
- **Attributes**: base_urls (array of strings), crawl_depth (integer), rate_limit (integer), excluded_patterns (array of strings)
- **Description**: Configuration parameters for the URL crawling process

### Embedding Configuration
- **Attributes**: model_name (string), api_key (string), batch_size (integer), dimension_size (integer)
- **Description**: Configuration parameters for the embedding generation process

## Assumptions

- The target book websites are publicly accessible and follow standard web practices
- Cohere API is available and responsive during pipeline execution
- Qdrant Cloud is available and responsive during pipeline execution
- The book content is primarily text-based with standard HTML structure
- The developer has valid API credentials for Cohere and Qdrant Cloud
- The free tier limits of Qdrant Cloud are sufficient for the intended use case
- The book content does not require special handling for multimedia elements

## Constraints

- Language: Python
- Embeddings: Cohere (latest stable embedding model)
- Vector DB: Qdrant Cloud (free tier)
- Content sources: Public website URLs only
- Must support incremental re-ingestion without duplication
- Execution: CLI-based or script-based pipeline
- Must operate within free tier limitations of external services

## Dependencies

- Cohere API for embedding generation
- Qdrant Cloud for vector storage
- Web scraping/crawling libraries for content extraction
- Python 3.x runtime environment