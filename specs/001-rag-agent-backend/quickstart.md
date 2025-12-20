# Quickstart Guide: RAG Agent Backend

## Prerequisites

- Python 3.11 or higher
- pip package manager
- Access to OpenAI API (API key)
- Access to Cohere API (API key)
- Access to Gemini API (API key) - optional
- Qdrant vector database instance (local or cloud)

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r backend/requirements.txt
```

### 4. Configure Environment Variables
Create a `.env` file in the backend directory:

```env
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
GEMINI_API_KEY=your_gemini_api_key  # Optional
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key  # Optional, depending on your Qdrant setup
QDRANT_COLLECTION_NAME=book_content
```

## Running the Service

### 1. Start the FastAPI Server
```bash
cd backend
uvicorn rag_agent.main:app --reload --host 0.0.0.0 --port 8000
```

### 2. Verify the Service is Running
Open your browser to `http://localhost:8000/docs` to access the interactive API documentation.

## Usage Examples

### 1. Query the RAG Agent
```bash
curl -X POST "http://localhost:8000/api/ask" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of humanoid robotics?",
    "provider": "openai",
    "max_chunks": 5
  }'
```

### 2. Retrieve Content Chunks
```bash
curl -X POST "http://localhost:8000/api/retrieve" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of humanoid robotics?",
    "max_chunks": 5
  }'
```

### 3. Check Service Health
```bash
curl -X GET "http://localhost:8000/health"
```

## Configuration Options

### Environment Variables
- `OPENAI_API_KEY`: API key for OpenAI services
- `COHERE_API_KEY`: API key for Cohere embedding services
- `GEMINI_API_KEY`: API key for Gemini services (optional)
- `QDRANT_URL`: URL for Qdrant vector database
- `QDRANT_API_KEY`: API key for Qdrant (if required)
- `QDRANT_COLLECTION_NAME`: Name of the collection in Qdrant to query

### Runtime Parameters
- `--host`: Host address to bind to (default: 0.0.0.0)
- `--port`: Port number to listen on (default: 8000)
- `--reload`: Enable auto-reload on code changes

## Troubleshooting

### Common Issues
1. **API Key Errors**: Verify all required API keys are correctly set in the `.env` file
2. **Qdrant Connection Issues**: Ensure the Qdrant URL and credentials are correct
3. **Performance Issues**: Check that the vector database has appropriate indexes

### Health Check
Use the `/health` endpoint to verify that all dependencies are accessible.

### Debugging
Enable debug logging by setting the `LOG_LEVEL` environment variable to `debug`:

```bash
export LOG_LEVEL=debug
```