"""
Pydantic models for API requests in the RAG Agent Backend
"""
from pydantic import BaseModel, Field
from typing import Optional


class AskRequest(BaseModel):
    """
    Request model for the ask endpoint
    """
    query: str = Field(
        ...,
        description="The user's question or query",
        min_length=1,
        max_length=10000,
        example="What are the key principles of humanoid robotics?"
    )
    provider: str = Field(
        "gemini",
        description="AI provider to use for answer generation",
        pattern="^gemini$"
    )
    max_chunks: int = Field(
        5,
        description="Maximum number of content chunks to retrieve",
        ge=1,
        le=20
    )
    user_id: Optional[str] = Field(
        None,
        description="Optional user identifier"
    )


class RetrieveRequest(BaseModel):
    """
    Request model for the retrieve endpoint
    """
    query: str = Field(
        ...,
        description="The query to search for relevant content",
        min_length=1,
        max_length=10000,
        example="What are the key principles of humanoid robotics?"
    )
    max_chunks: int = Field(
        5,
        description="Maximum number of content chunks to retrieve",
        ge=1,
        le=20
    )