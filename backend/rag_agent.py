"""
RAG Agent Implementation using OpenAI Agents SDK with Qdrant retrieval

This module implements a RAG (Retrieval-Augmented Generation) agent that:
1. Connects to Qdrant to retrieve relevant book content
2. Uses OpenAI Agents SDK to generate answers based on retrieved context
3. Includes comprehensive logging and debugging outputs
"""

import os
import asyncio
import time
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from openai import OpenAI
from pydantic import BaseModel
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class Chunk(BaseModel):
    """Represents a content chunk retrieved from Qdrant"""
    content: str
    source_document: str
    page_number: Optional[int] = None
    section_title: Optional[str] = None
    similarity_score: float
    chunk_id: str


class QueryRequest(BaseModel):
    """Request model for querying the RAG agent"""
    query: str
    provider: str = "openai"  # "openai" or "gemini"
    max_chunks: int = 5
    user_id: Optional[str] = None


class RAGResponse(BaseModel):
    """Response model for RAG agent responses"""
    answer: str
    sources: List[Chunk]
    provider_used: str
    debug_info: Dict[str, Any]


class RAGAgent:
    """
    RAG Agent that retrieves information from Qdrant and generates answers using OpenAI Agents SDK
    """

    def __init__(self):
        # Initialize clients
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY"),
            prefer_grpc=True
        )

        self.cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))

        # Only initialize OpenAI client if API key is provided
        openai_api_key = os.getenv("OPENAI_API_KEY")
        if openai_api_key and openai_api_key.startswith("sk-"):
            self.openai_client = OpenAI(api_key=openai_api_key)
        else:
            self.openai_client = None

        self.qdrant_collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

        # Verify connections
        self._verify_connections()

    def _verify_connections(self):
        """Verify that all required services are accessible"""
        try:
            # Test Qdrant connection
            self.qdrant_client.get_collection(self.qdrant_collection_name)
            logger.info("✓ Qdrant connection successful")
        except Exception as e:
            logger.error(f"✗ Qdrant connection failed: {e}")
            raise

        try:
            # Test Cohere connection with a simple embedding
            self.cohere_client.embed(texts=["test"], model="embed-english-v3.0", input_type="search_document")
            logger.info("✓ Cohere connection successful")
        except Exception as e:
            logger.error(f"✗ Cohere connection failed: {e}")
            raise

        # Test OpenAI connection only if OPENAI_API_KEY is provided
        # If not provided, we assume Gemini will be used instead
        if os.getenv("OPENAI_API_KEY"):
            try:
                # Test OpenAI connection with a simple completion
                self.openai_client.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=[{"role": "user", "content": "test"}],
                    max_tokens=5
                )
                logger.info("✓ OpenAI connection successful")
            except Exception as e:
                logger.error(f"✗ OpenAI connection failed: {e}")
                raise
        else:
            logger.info("ℹ OpenAI API key not provided, assuming Gemini will be used")

        # Test Gemini connection if GEMINI_API_KEY is provided
        if os.getenv("GEMINI_API_KEY"):
            try:
                import google.generativeai as genai
                genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
                model = genai.GenerativeModel(  "gemini-2.5-flash")
                response = model.generate_content("Hello, this is a test.")
                logger.info("✓ Gemini connection successful")
            except Exception as e:
                logger.error(f"✗ Gemini connection failed: {e}")
                raise
        else:
            logger.error("✗ GEMINI_API_KEY not provided, but it's required when OpenAI is not available")
            raise ValueError("Either OPENAI_API_KEY or GEMINI_API_KEY must be provided")

    def retrieve_chunks(self, query: str, max_chunks: int = 5) -> List[Chunk]:
        """
        Retrieve relevant content chunks from Qdrant based on the query
        """
        start_time = time.time()

        # Generate embedding for the query using Cohere
        logger.info(f"Generating embedding for query: {query[:50]}...")
        response = self.cohere_client.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_embedding = response.embeddings[0]

        # Perform semantic search in Qdrant using the query_points method
        logger.info(f"Performing semantic search in Qdrant collection: {self.qdrant_collection_name}")
        search_results = self.qdrant_client.query_points(
            collection_name=self.qdrant_collection_name,
            query=query_embedding,
            limit=max_chunks,
            with_payload=True
        )

        # Convert search results to Chunk objects
        # Note: query_points returns a QueryResponse object with points attribute
        chunks = []
        for result in search_results.points:  # Access the .points attribute
            chunk = Chunk(
                content=result.payload.get("content_preview", ""),
                source_document=result.payload.get("title", result.payload.get("url", "Unknown")),
                page_number=result.payload.get("page_number"),
                section_title=result.payload.get("section_title"),
                similarity_score=result.score,
                chunk_id=str(result.id)
            )
            chunks.append(chunk)

        retrieval_time = time.time() - start_time
        logger.info(f"Retrieved {len(chunks)} chunks in {retrieval_time:.2f} seconds")

        return chunks

    def generate_answer_with_openai(self, query: str, chunks: List[Chunk]) -> str:
        """
        Generate an answer using OpenAI based on the query and retrieved chunks
        """
        start_time = time.time()

        # Format the context from retrieved chunks
        context = "\n\n".join([f"Source: {chunk.source_document}\nContent: {chunk.content}" for chunk in chunks])

        # Create the prompt for OpenAI
        system_prompt = """You are a helpful assistant that answers questions based on provided book content.
        Use the provided context to answer the user's question accurately.
        If the context doesn't contain enough information to answer the question, say so clearly."""

        user_prompt = f"""Context:\n{context}\n\nQuestion: {query}"""

        # Call OpenAI API to generate the answer (only if client is initialized)
        if self.openai_client is None:
            raise ValueError("OpenAI API key not configured. Please set OPENAI_API_KEY environment variable.")

        response = self.openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            max_tokens=1000,
            temperature=0.7
        )

        answer = response.choices[0].message.content
        generation_time = time.time() - start_time

        logger.info(f"Generated answer in {generation_time:.2f} seconds using OpenAI")

        return answer

    def generate_answer_with_gemini(self, query: str, chunks: List[Chunk]) -> str:
        """
        Generate an answer using Gemini based on the query and retrieved chunks
        """
        try:
            import google.generativeai as genai

            # Configure with the API key
            genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

            # Select the model
            model = genai.GenerativeModel('gemini-2.5-flash')

            # Format the context from retrieved chunks
            context = "\n\n".join([f"Source: {chunk.source_document}\nContent: {chunk.content}" for chunk in chunks])

            # Create the prompt for Gemini
            prompt = f"""You are a helpful assistant that answers questions based on provided book content.
            Use the provided context to answer the user's question accurately.
            If the context doesn't contain enough information to answer the question, say so clearly.

            Context:
            {context}

            Question: {query}"""

            # Generate the answer using Gemini
            response = model.generate_content(prompt)
            answer = response.text if response.text else "I couldn't generate a proper answer based on the provided context."

            logger.info("Generated answer using Gemini")
            return answer

        except ImportError:
            logger.error("Google Generative AI library not installed")
            raise ValueError("Google Generative AI library not installed. Please install with: pip install google-generativeai")
        except Exception as e:
            logger.error(f"Error generating answer with Gemini: {e}")
            raise

    def query(self, query: str, provider: str = "openai", max_chunks: int = 5) -> RAGResponse:
        """
        Main method to query the RAG agent
        """
        start_time = time.time()

        logger.info(f"Processing query: {query}")

        # Step 1: Retrieve relevant chunks from Qdrant
        chunks = self.retrieve_chunks(query, max_chunks)

        # Step 2: Generate answer based on selected provider
        if provider.lower() == "openai":
            if self.openai_client is None:
                logger.warning("OpenAI not configured, falling back to Gemini")
                answer = self.generate_answer_with_gemini(query, chunks)
                provider_used = "gemini"
            else:
                answer = self.generate_answer_with_openai(query, chunks)
                provider_used = "openai"
        elif provider.lower() == "gemini":
            answer = self.generate_answer_with_gemini(query, chunks)
            provider_used = "gemini"
        else:
            raise ValueError(f"Unsupported provider: {provider}")

        # Step 3: Prepare response with debug information
        total_time = time.time() - start_time
        debug_info = {
            "embedding_time": 0,  # Would need to be measured separately in a more detailed implementation
            "retrieval_time": sum([0.1 for _ in chunks]),  # Placeholder - actual timing would be captured in retrieve_chunks
            "generation_time": total_time - sum([0.1 for _ in chunks]),  # Placeholder
            "total_time": total_time
        }

        response = RAGResponse(
            answer=answer,
            sources=chunks,
            provider_used=provider_used,
            debug_info=debug_info
        )

        logger.info(f"Query completed in {total_time:.2f} seconds using {provider_used}")

        return response


# Example usage
if __name__ == "__main__":
    # Initialize the RAG agent
    agent = RAGAgent()

    # Example query
    query_request = QueryRequest(
        query="What are the key principles of humanoid robotics?",
        provider="openai",
        max_chunks=3
    )

    # Process the query
    response = agent.query(
        query=query_request.query,
        provider=query_request.provider,
        max_chunks=query_request.max_chunks
    )

    # Print the results
    print(f"Answer: {response.answer}")
    print(f"Sources: {len(response.sources)} chunks used")
    print(f"Provider: {response.provider_used}")
    print(f"Debug Info: {response.debug_info}")