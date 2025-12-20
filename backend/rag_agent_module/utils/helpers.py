"""
Helper utilities for the RAG Agent Backend
"""
import time
import logging
from typing import List, Dict, Any, Optional
from ..models.response_models import Chunk
from qdrant_client.http import models


def format_chunks_for_agent(chunks: List[Dict]) -> List[Chunk]:
    """
    Format raw chunk data from Qdrant into Chunk models
    """
    formatted_chunks = []
    for chunk_data in chunks:
        # Extract fields from Qdrant payload
        payload = chunk_data.get('payload', {})

        chunk = Chunk(
            content=payload.get('content', ''),
            source_document=payload.get('source_document', 'Unknown'),
            page_number=payload.get('page_number'),
            section_title=payload.get('section_title'),
            similarity_score=chunk_data.get('score', 0.0),
            chunk_id=str(chunk_data.get('id', ''))
        )
        formatted_chunks.append(chunk)

    return formatted_chunks


def create_qdrant_vector_query(query_vector: List[float], top_k: int = 5) -> models.SearchRequest:
    """
    Create a Qdrant search request for vector similarity search
    """
    return models.SearchRequest(
        vector=query_vector,
        limit=top_k,
        with_payload=True,
        with_vectors=False
    )


def measure_time(func):
    """
    Decorator to measure execution time of functions
    """
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        logging.info(f"{func.__name__} executed in {execution_time:.4f} seconds")
        return result, execution_time
    return wrapper


def validate_query_text(query: str) -> bool:
    """
    Validate query text meets requirements
    """
    if not query or len(query.strip()) == 0:
        return False
    if len(query) > 10000:  # Max length from API contract
        return False
    return True


def validate_max_chunks(max_chunks: int) -> bool:
    """
    Validate max_chunks parameter
    """
    return 1 <= max_chunks <= 20


def calculate_similarity_threshold(score: float, threshold: float = 0.3) -> bool:
    """
    Determine if similarity score meets threshold for relevance
    """
    return score >= threshold


def sanitize_input(text: str) -> str:
    """
    Sanitize input text to prevent injection attacks
    """
    # Remove potentially harmful characters/sequences
    sanitized = text.replace('\0', '')  # Remove null bytes
    return sanitized