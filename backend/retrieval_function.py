"""
Qdrant Embedding Retrieval Function

This script provides functionality to retrieve data from Qdrant using embeddings.
It connects to Qdrant Cloud using provided credentials and performs semantic search.
"""

import os
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class QdrantRetrieval:
    def __init__(self, qdrant_url=None, qdrant_api_key=None, collection_name="RAG_embeddings"):
        """
        Initialize the Qdrant retrieval client.

        Args:
            qdrant_url (str): Qdrant Cloud URL
            qdrant_api_key (str): Qdrant API key
            collection_name (str): Name of the collection to search in
        """
        # Use provided parameters or fall back to environment variables
        self.qdrant_url = qdrant_url or os.getenv('QDRANT_URL')
        self.qdrant_api_key = qdrant_api_key or os.getenv('QDRANT_API_KEY')
        self.collection_name = collection_name or os.getenv('QDRANT_COLLECTION_NAME', 'RAG_embeddings')

        if not self.qdrant_url or not self.qdrant_api_key:
            raise ValueError("Qdrant URL and API key are required")

        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key,
        )

        # Initialize Cohere client for generating query embeddings
        cohere_api_key = os.getenv('COHERE_API_KEY')
        if not cohere_api_key:
            raise ValueError("Cohere API key is required")

        self.cohere_client = cohere.Client(cohere_api_key)

        logger.info(f"Qdrant retrieval client initialized for collection: {self.collection_name}")

    def get_embedding(self, text):
        """
        Generate embedding for the given text using Cohere.

        Args:
            text (str): Text to generate embedding for

        Returns:
            list: Embedding vector
        """
        try:
            response = self.cohere_client.embed(
                texts=[text],
                model="embed-english-v3.0",
                input_type="search_query"  # Using search_query for better retrieval
            )
            return response.embeddings[0]
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            return None

    def retrieve_similar(self, query_text, top_k=10, filters=None):
        """
        Retrieve similar documents from Qdrant based on the query text.

        Args:
            query_text (str): Query text to search for
            top_k (int): Number of results to return
            filters (dict, optional): Additional filters for the search

        Returns:
            list: List of similar documents with metadata
        """
        # Generate embedding for the query
        query_embedding = self.get_embedding(query_text)
        if query_embedding is None:
            return []

        try:
            # Perform semantic search in Qdrant using the correct API for newer qdrant-client
            # For qdrant-client 1.16.2+, we need to use query_points method
            from qdrant_client.http import models

            search_result = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                with_payload=True,
                with_vectors=False,
            )

            results = []
            for point in search_result.points:
                result = {
                    'id': point.id,
                    'score': point.score,
                    'payload': point.payload,
                    'content': point.payload.get('content_preview', '') if point.payload else ''
                }
                results.append(result)

            logger.info(f"Retrieved {len(results)} results for query: '{query_text}'")
            return results

        except Exception as e:
            logger.error(f"Error retrieving similar documents: {e}")
            return []

    def retrieve_with_filters(self, query_text, top_k=10, filters=None):
        """
        Retrieve similar documents with additional filters.

        Args:
            query_text (str): Query text to search for
            top_k (int): Number of results to return
            filters (dict, optional): Additional filters for the search

        Returns:
            list: List of similar documents with metadata
        """
        # Generate embedding for the query
        query_embedding = self.get_embedding(query_text)
        if query_embedding is None:
            return []

        try:
            # Build the search filter if provided
            search_filter = None
            if filters:
                conditions = []
                for key, value in filters.items():
                    conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )
                if conditions:
                    search_filter = models.Filter(must=conditions)

            # Perform semantic search in Qdrant with filters using the correct API for newer qdrant-client
            search_result = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                query_filter=search_filter,
                with_payload=True,
                with_vectors=False,
            )

            results = []
            for point in search_result.points:
                result = {
                    'id': point.id,
                    'score': point.score,
                    'payload': point.payload,
                    'content': point.payload.get('content_preview', '') if point.payload else ''
                }
                results.append(result)

            logger.info(f"Retrieved {len(results)} results with filters for query: '{query_text}'")
            return results

        except Exception as e:
            logger.error(f"Error retrieving documents with filters: {e}")
            return []

    def get_all_documents(self, limit=100):
        """
        Retrieve all documents from the collection (useful for debugging).

        Args:
            limit (int): Maximum number of documents to retrieve

        Returns:
            list: List of documents with metadata
        """
        try:
            # Scroll through the collection to get documents
            records, _ = self.qdrant_client.scroll(
                collection_name=self.collection_name,
                limit=limit,
                with_payload=True,
                with_vectors=False,
            )

            results = []
            for record in records:
                result = {
                    'id': record.id,
                    'payload': record.payload,
                    'content': record.payload.get('content_preview', '') if record.payload else ''
                }
                results.append(result)

            logger.info(f"Retrieved {len(results)} documents from collection")
            return results

        except Exception as e:
            logger.error(f"Error retrieving all documents: {e}")
            return []

    def check_collection_health(self):
        """
        Check the health and status of the Qdrant collection.

        Returns:
            dict: Collection information
        """
        try:
            collection_info = self.qdrant_client.get_collection(self.collection_name)
            health_info = {
                'collection_name': self.collection_name,
                'vector_count': collection_info.points_count,
                'vector_dimensions': collection_info.config.params.vectors.size,
                'distance_type': collection_info.config.params.vectors.distance
            }
            logger.info(f"Collection health: {health_info}")
            return health_info
        except Exception as e:
            logger.error(f"Error checking collection health: {e}")
            return None

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        # Clean up resources if needed
        logger.info("Qdrant retrieval client closed")


def main():
    """
    Example usage of the Qdrant retrieval functionality.
    """
    logger.info("Initializing Qdrant retrieval system...")

    # Initialize the retrieval client
    # You can pass the credentials directly or use environment variables
    try:
        retrieval = QdrantRetrieval(
            # Uncomment and provide your credentials directly if needed:
            # qdrant_url="your_qdrant_url",
            # qdrant_api_key="your_qdrant_api_key",
            # collection_name="your_collection_name"
        )

        # Check collection health
        health_info = retrieval.check_collection_health()
        if health_info:
            print(f"Collection Health: {health_info}")

        # Example queries
        test_queries = [
            "What is embodied intelligence?",
            "Explain humanoid robot control systems",
            "How does reinforcement learning apply to robotics?",
            "What are the foundations of physical AI?"
        ]

        # Test retrieval for each query
        for query in test_queries:
            print(f"\nQuery: {query}")
            results = retrieval.retrieve_similar(query, top_k=5)

            if results:
                print(f"Found {len(results)} results:")
                for i, result in enumerate(results[:3]):  # Show top 3
                    print(f"  {i+1}. Score: {result['score']:.4f}")
                    print(f"     URL: {result['payload'].get('url', 'N/A')}")
                    print(f"     Title: {result['payload'].get('title', 'N/A')}")
                    print(f"     Content Preview: {result['content'][:100]}...")
                    print()
            else:
                print("  No results found for this query.")

    except Exception as e:
        logger.error(f"Error in main: {e}")


if __name__ == "__main__":
    main()