"""
Main FastAPI application for the RAG Agent Backend
"""
from fastapi import FastAPI, HTTPException, Depends
from typing import Optional
import time
import datetime
from .models.request_models import AskRequest, RetrieveRequest
from .models.response_models import AskResponse, RetrieveResponse, HealthResponse
from .services.embedding_service import EmbeddingService
from .services.retrieval_service import RetrievalService
from .services.agent_service import AgentService
from .services.health_service import HealthService
from .config.logger import logger
from .utils.helpers import format_chunks_for_agent


# Initialize FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="A Retrieval-Augmented Generation agent that answers questions based on book content",
    version="1.0.0"
)

# Services will be initialized per request to avoid startup errors when API keys are not available


@app.post("/api/ask", response_model=AskResponse)
async def ask_endpoint(request: AskRequest):
    """
    Submit a query to the RAG agent to get a grounded answer based on book content.
    """
    start_time = time.time()

    # Initialize services
    embedding_service = EmbeddingService()
    retrieval_service = RetrievalService()
    agent_service = AgentService()

    try:
        # Validate inputs
        if not request.query.strip():
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        if request.max_chunks < 1 or request.max_chunks > 20:
            raise HTTPException(status_code=400, detail="max_chunks must be between 1 and 20")

        if not agent_service.validate_provider(request.provider):
            raise HTTPException(status_code=400, detail="Provider must be 'gemini'")

        # Generate embedding for the query
        embedding_start = time.time()
        query_embedding = embedding_service.generate_query_embedding(request.query)
        embedding_time = time.time() - embedding_start

        # Retrieve relevant chunks from Qdrant
        retrieval_start = time.time()
        raw_chunks = retrieval_service.retrieve_chunks(query_embedding, request.max_chunks)
        retrieval_time = time.time() - retrieval_start

        # Format chunks for the response
        formatted_chunks = format_chunks_for_agent(raw_chunks)

        # Generate answer using the selected provider
        generation_start = time.time()
        answer = agent_service.generate_answer(
            query=request.query,
            chunks=formatted_chunks,
            provider=request.provider,
            fallback_to_alternate=True
        )
        generation_time = time.time() - generation_start

        # Calculate total time
        total_time = time.time() - start_time

        # Create response
        response = AskResponse(
            answer=answer,
            sources=formatted_chunks,
            provider_used=request.provider,
            debug_info={
                "embedding_time": embedding_time,
                "retrieval_time": retrieval_time,
                "generation_time": generation_time,
                "total_time": total_time
            }
        )

        logger.info(f"Query processed successfully in {total_time:.2f}s using {request.provider}")
        return response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing ask request: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/retrieve", response_model=RetrieveResponse)
async def retrieve_endpoint(request: RetrieveRequest):
    """
    Retrieve relevant content chunks from the vector database based on a query.
    """
    start_time = time.time()

    # Initialize services
    embedding_service = EmbeddingService()
    retrieval_service = RetrievalService()

    try:
        # Validate inputs
        if not request.query.strip():
            raise HTTPException(status_code=400, detail="Query cannot be empty")

        if request.max_chunks < 1 or request.max_chunks > 20:
            raise HTTPException(status_code=400, detail="max_chunks must be between 1 and 20")

        # Generate embedding for the query
        query_embedding = embedding_service.generate_query_embedding(request.query)

        # Retrieve relevant chunks from Qdrant
        raw_chunks = retrieval_service.retrieve_chunks(query_embedding, request.max_chunks)

        # Format chunks for the response
        formatted_chunks = format_chunks_for_agent(raw_chunks)

        # Calculate retrieval time
        retrieval_time = time.time() - start_time

        # Create response
        response = RetrieveResponse(
            chunks=formatted_chunks,
            retrieval_time=retrieval_time
        )

        logger.info(f"Retrieved {len(formatted_chunks)} chunks in {retrieval_time:.2f}s")
        return response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing retrieve request: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Check the health status of the RAG agent service.
    """
    # Initialize services
    health_service = HealthService()

    try:
        health_info = health_service.get_health_status()
        return HealthResponse(**health_info)
    except Exception as e:
        logger.error(f"Error during health check: {e}")
        raise HTTPException(status_code=500, detail=str(e))


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