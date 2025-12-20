# RAG Agent Backend

A Retrieval-Augmented Generation (RAG) agent that answers questions based on book content using vector search and AI models.

## Overview

This backend service provides a RAG (Retrieval-Augmented Generation) agent that:
- Accepts user queries about book content
- Uses Cohere to generate embeddings for semantic search
- Performs vector search in Qdrant to retrieve relevant content chunks
- Generates grounded answers using OpenAI or Gemini models
- Provides comprehensive debugging outputs and performance metrics

## Architecture

The service is built with:
- **FastAPI**: Web framework with automatic API documentation
- **Cohere**: Embedding generation for semantic search
- **Qdrant**: Vector database for content retrieval
- **OpenAI/Gemini**: Language models for answer generation
- **Pydantic**: Data validation and serialization

## Setup

### Prerequisites

- Python 3.11+
- Access to OpenAI API (API key)
- Access to Cohere API (API key)
- Access to Gemini API (API key) - optional
- Qdrant vector database instance (local or cloud)

### Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r backend/requirements.txt
   ```

4. Configure environment variables:
   Create a `.env` file in the backend directory based on `.env.example`:
   ```bash
   cp backend/.env.example backend/.env
   # Edit backend/.env with your actual API keys and configuration
   ```

### Environment Variables

- `OPENAI_API_KEY`: API key for OpenAI services
- `GEMINI_API_KEY`: API key for Gemini services
- `COHERE_API_KEY`: API key for Cohere embedding services
- `QDRANT_URL`: URL for Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant (if required)
- `QDRANT_COLLECTION_NAME`: Name of the collection in Qdrant to query
- `LOG_LEVEL`: Logging level (default: INFO)
- `HOST`: Host address to bind to (default: 0.0.0.0)
- `PORT`: Port number to listen on (default: 8000)

## Usage

### Running the Service

```bash
cd backend
uvicorn rag_agent.main:app --reload --host 0.0.0.0 --port 8000
```

The API documentation will be available at `http://localhost:8000/docs`.

### API Endpoints

#### Ask Endpoint
`POST /api/ask` - Submit a query to get a grounded answer

Request body:
```json
{
  "query": "What are the key principles of humanoid robotics?",
  "provider": "openai",
  "max_chunks": 5,
  "user_id": "optional-user-id"
}
```

Response:
```json
{
  "answer": "The key principles of humanoid robotics...",
  "sources": [...],
  "provider_used": "openai",
  "debug_info": {
    "embedding_time": 0.123,
    "retrieval_time": 0.456,
    "generation_time": 1.234,
    "total_time": 1.813
  }
}
```

#### Retrieve Endpoint
`POST /api/retrieve` - Retrieve content chunks without generating an answer

Request body:
```json
{
  "query": "What are the key principles of humanoid robotics?",
  "max_chunks": 5
}
```

#### Health Check
`GET /health` - Check the health status of the service

### Configuration

The system supports switching between OpenAI and Gemini models via the `provider` parameter in API requests. If one provider fails, the system can automatically fall back to the other if configured.

## Development

### Project Structure

```
backend/
├── rag_agent/              # Main application package
│   ├── __init__.py
│   ├── main.py             # FastAPI application
│   ├── config/             # Configuration and settings
│   │   ├── __init__.py
│   │   ├── settings.py     # Settings management
│   │   └── logger.py       # Logging configuration
│   ├── models/             # Pydantic models
│   │   ├── __init__.py
│   │   ├── request_models.py
│   │   └── response_models.py
│   ├── services/           # Business logic services
│   │   ├── __init__.py
│   │   ├── embedding_service.py
│   │   ├── retrieval_service.py
│   │   ├── agent_service.py
│   │   └── health_service.py
│   └── utils/              # Utility functions
│       ├── __init__.py
│       └── helpers.py
├── requirements.txt        # Dependencies
├── .env.example           # Environment variables example
└── README.md              # This file
```

## Performance

The system is designed to respond to queries within 1.2 seconds under normal load conditions, with:
- Embedding generation: <200ms p95
- Retrieval: <500ms p95
- Answer generation: <500ms p95

## Error Handling

The system handles various error conditions:
- Unavailable external services (Qdrant, OpenAI, Gemini)
- Invalid queries or parameters
- Rate limiting from external APIs
- Malformed requests