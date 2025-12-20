"""
Pydantic models for API responses in the RAG Agent Backend
"""
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from enum import Enum


class ProviderType(str, Enum):
    """
    Enum for AI provider types
    """
    gemini = "gemini"


class Chunk(BaseModel):
    """
    Model representing a content chunk retrieved from Qdrant
    """
    content: str = Field(
        ...,
        description="The retrieved content chunk"
    )
    source_document: str = Field(
        ...,
        description="Reference to the original document"
    )
    page_number: Optional[int] = Field(
        None,
        description="Page number in the original document"
    )
    section_title: Optional[str] = Field(
        None,
        description="Title of the section containing the chunk"
    )
    similarity_score: float = Field(
        ...,
        description="Cosine similarity score to the query",
        ge=0.0,
        le=1.0
    )
    chunk_id: str = Field(
        ...,
        description="Unique identifier for the chunk"
    )


class DebugInfo(BaseModel):
    """
    Model for debugging information with timing details
    """
    embedding_time: float = Field(
        ...,
        description="Time taken for embedding generation in seconds"
    )
    retrieval_time: float = Field(
        ...,
        description="Time taken for retrieval in seconds"
    )
    generation_time: float = Field(
        ...,
        description="Time taken for answer generation in seconds"
    )
    total_time: float = Field(
        ...,
        description="Total processing time in seconds"
    )


class AskResponse(BaseModel):
    """
    Response model for the ask endpoint
    """
    answer: str = Field(
        ...,
        description="The generated answer based on retrieved context"
    )
    sources: List[Chunk] = Field(
        ...,
        description="List of content chunks used to generate the answer"
    )
    provider_used: ProviderType = Field(
        ...,
        description="Which AI provider was used to generate the answer"
    )
    debug_info: DebugInfo = Field(
        ...,
        description="Debugging information with timing details"
    )


class RetrieveResponse(BaseModel):
    """
    Response model for the retrieve endpoint
    """
    chunks: List[Chunk] = Field(
        ...,
        description="List of retrieved content chunks"
    )
    retrieval_time: float = Field(
        ...,
        description="Time taken for retrieval in seconds"
    )


class HealthStatus(str, Enum):
    """
    Enum for health status
    """
    healthy = "healthy"
    unhealthy = "unhealthy"


class DependencyStatus(str, Enum):
    """
    Enum for dependency status
    """
    connected = "connected"
    disconnected = "disconnected"


class HealthResponse(BaseModel):
    """
    Response model for the health endpoint
    """
    status: HealthStatus = Field(
        ...,
        description="Health status of the service"
    )
    timestamp: str = Field(
        ...,
        description="Timestamp of the health check"
    )
    dependencies: Dict[str, DependencyStatus] = Field(
        ...,
        description="Status of external dependencies"
    )


class ErrorResponse(BaseModel):
    """
    Response model for error responses
    """
    error: str = Field(
        ...,
        description="Error message describing the issue"
    )
    details: Optional[Dict[str, Any]] = Field(
        None,
        description="Additional error details"
    )