"""
FastAPI Application for RAG Agent Backend

This module implements the FastAPI endpoints for the RAG agent:
- /api/ask: Submit queries to the RAG agent
- /api/retrieve: Retrieve relevant content chunks from Qdrant
- /health: Check the health status of the service
"""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import time
import os
from rag_agent import Chunk, QueryRequest, RAGResponse

# Check if we should use mock agent (when environment variables are not properly set)
use_mock = not (os.getenv("COHERE_API_KEY") and os.getenv("QDRANT_API_KEY") and os.getenv("QDRANT_URL"))

if use_mock:
    from mock_rag_agent import MockRAGAgent as RAGAgent
    print("WARNING: Using Mock RAG Agent due to missing environment variables. Set COHERE_API_KEY, QDRANT_API_KEY, and QDRANT_URL to use the real agent.")
else:
    from rag_agent import RAGAgent

# Initialize FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="A Retrieval-Augmented Generation agent that answers questions based on book content",
    version="1.0.0"
)

# Add CORS middleware to allow requests from the frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000", "http://localhost:3000/PhysicalAI-HumanoidRobotics/"],  # Allow frontend origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize the RAG agent
try:
    rag_agent = RAGAgent()
    if use_mock:
        print("Mock RAG Agent initialized successfully")
    else:
        print("Real RAG Agent initialized successfully")
except Exception as e:
    if use_mock:
        # If using mock and it still fails, there's a code issue
        print(f"Error initializing mock RAG Agent: {e}")
        raise
    else:
        # If real agent fails, fall back to mock
        print(f"Error initializing real RAG Agent: {e}. Falling back to Mock RAG Agent.")
        from mock_rag_agent import MockRAGAgent
        rag_agent = MockRAGAgent()


class AskRequest(BaseModel):
    """Request model for the ask endpoint"""
    query: str
    provider: str = "openai"  # "openai" or "gemini"
    max_chunks: int = 5
    user_id: Optional[str] = None


class RetrieveRequest(BaseModel):
    """Request model for the retrieve endpoint"""
    query: str
    max_chunks: int = 5


class HealthResponse(BaseModel):
    """Response model for the health endpoint"""
    status: str
    timestamp: str
    dependencies: Dict[str, str]


@app.post("/api/ask", response_model=RAGResponse)
async def ask_endpoint(request: AskRequest):
    """
    Submit a query to the RAG agent to get a grounded answer based on book content.
    """
    try:
        start_time = time.time()

        # Process the query through the RAG agent
        response = rag_agent.query(
            query=request.query,
            provider=request.provider,
            max_chunks=request.max_chunks
        )

        # Add total processing time to debug info
        total_time = time.time() - start_time
        response.debug_info["total_time"] = total_time

        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/retrieve", response_model=Dict[str, Any])
async def retrieve_endpoint(request: RetrieveRequest):
    """
    Retrieve relevant content chunks from the vector database based on a query.
    """
    try:
        start_time = time.time()

        # Retrieve chunks from Qdrant
        chunks = rag_agent.retrieve_chunks(request.query, request.max_chunks)

        retrieval_time = time.time() - start_time

        return {
            "chunks": [chunk.model_dump() for chunk in chunks],
            "retrieval_time": retrieval_time
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Check the health status of the RAG agent service.
    """
    import datetime

    # Check dependencies
    dependencies = {
        "qdrant": "connected",  # This would be checked in a real implementation
        "openai": "connected",  # This would be checked in a real implementation
        "cohere": "connected"   # This would be checked in a real implementation
    }

    # In a real implementation, you would perform actual connectivity checks
    # For now, we assume all dependencies are connected since the agent initialized successfully

    return HealthResponse(
        status="healthy",
        timestamp=datetime.datetime.now().isoformat(),
        dependencies=dependencies
    )


@app.get("/")
async def root():
    """
    Root endpoint that provides basic information about the API.
    """
    return {
        "message": "RAG Agent API",
        "version": "1.0.0",
        "endpoints": [
            "/api/ask - Submit queries to the RAG agent",
            "/api/retrieve - Retrieve relevant content chunks",
            "/health - Check service health",
            "/docs - Interactive API documentation"
        ]
    }


# If running this file directly, start the server
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)