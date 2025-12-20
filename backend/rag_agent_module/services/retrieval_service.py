"""
Retrieval service for the RAG Agent Backend using Qdrant
"""
import time
from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.http import models
from ..config.settings import settings
from ..config.logger import logger
from ..utils.helpers import format_chunks_for_agent


class RetrievalService:
    """
    Service class for retrieving content from Qdrant vector database
    """
    def __init__(self):
        self.qdrant_client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            prefer_grpc=True
        )
        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self.logger = logger

    def retrieve_chunks(self, query_embedding: List[float], max_chunks: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content chunks from Qdrant based on query embedding
        """
        start_time = time.time()

        try:
            # Perform semantic search in Qdrant - using the available API method
            # Different versions of Qdrant client have different method names
            import inspect
            available_methods = [method for method in dir(self.qdrant_client) if 'search' in method.lower()]

            if hasattr(self.qdrant_client, 'search'):
                search_results = self.qdrant_client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    limit=max_chunks,
                    with_payload=True,
                    score_threshold=0.3  # Minimum similarity threshold
                )
            elif hasattr(self.qdrant_client, 'search_points'):
                search_results = self.qdrant_client.search_points(
                    collection_name=self.collection_name,
                    vector=query_embedding,
                    limit=max_chunks,
                    with_payload=True,
                    score_threshold=0.3  # Minimum similarity threshold
                )
            else:
                # If neither method exists, try to use the points API directly
                # This may be an older version or different API structure
                raise AttributeError(f"Qdrant client does not have 'search' or 'search_points' method. Available methods with 'search': {available_methods}")

            # Format results
            chunks = []
            for result in search_results:
                chunk_data = {
                    'id': result.id,
                    'payload': result.payload,
                    'score': result.score
                }
                chunks.append(chunk_data)

            retrieval_time = time.time() - start_time
            self.logger.info(f"Retrieved {len(chunks)} chunks in {retrieval_time:.2f} seconds")

            return chunks

        except Exception as e:
            self.logger.error(f"Error retrieving chunks from Qdrant: {e}")
            raise

    def verify_collection_exists(self) -> bool:
        """
        Verify that the specified collection exists in Qdrant
        """
        try:
            collections = self.qdrant_client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)
            return collection_exists
        except Exception as e:
            self.logger.error(f"Error checking collection existence: {e}")
            return False

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection
        """
        try:
            collection_info = self.qdrant_client.get_collection(self.collection_name)
            return {
                "name": collection_info.config.params.vectors_count,
                "vectors_count": collection_info.config.params.vectors_count,
                "indexed_vectors_count": collection_info.indexed_vectors_count
            }
        except Exception as e:
            self.logger.error(f"Error getting collection info: {e}")
            return {}

    def search_with_filters(self, query_embedding: List[float], filters: Dict[str, Any], max_chunks: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve chunks with additional filters
        """
        start_time = time.time()

        try:
            # Convert filters to Qdrant filter format
            qdrant_filter = self._convert_to_qdrant_filter(filters)

            # Perform semantic search in Qdrant with filters - using grpc or http API depending on version
            try:
                # Try the newer search method first
                search_results = self.qdrant_client.search(
                    collection_name=self.collection_name,
                    query_vector=query_embedding,
                    limit=max_chunks,
                    with_payload=True,
                    query_filter=qdrant_filter,
                    score_threshold=0.3
                )
            except AttributeError:
                # Fallback to older method if search doesn't exist
                search_results = self.qdrant_client.search_points(
                    collection_name=self.collection_name,
                    vector=query_embedding,
                    limit=max_chunks,
                    with_payload=True,
                    filter=qdrant_filter,
                    score_threshold=0.3
                )

            chunks = []
            for result in search_results:
                chunk_data = {
                    'id': result.id,
                    'payload': result.payload,
                    'score': result.score
                }
                chunks.append(chunk_data)

            retrieval_time = time.time() - start_time
            self.logger.info(f"Retrieved {len(chunks)} filtered chunks in {retrieval_time:.2f} seconds")

            return chunks

        except Exception as e:
            self.logger.error(f"Error retrieving filtered chunks from Qdrant: {e}")
            raise

    def _convert_to_qdrant_filter(self, filters: Dict[str, Any]) -> models.Filter:
        """
        Convert dictionary filters to Qdrant Filter format
        """
        conditions = []
        for key, value in filters.items():
            conditions.append(
                models.FieldCondition(
                    key=key,
                    match=models.MatchValue(value=value)
                )
            )

        return models.Filter(must=conditions)

    def retrieve_content_tool(self, query: str, max_chunks: int = 5) -> str:
        """
        Tool function for the OpenAI agent to retrieve content from Qdrant
        Returns formatted content as a string for the agent to use
        """
        try:
            # First, get embeddings for the query using the embedding service
            from .embedding_service import EmbeddingService
            embedding_service = EmbeddingService()
            query_embedding = embedding_service.generate_query_embedding(query)

            # Retrieve relevant chunks
            chunks = self.retrieve_chunks(query_embedding, max_chunks)

            # Format the results for the agent
            formatted_results = []
            for chunk in chunks:
                payload = chunk['payload']
                content = payload.get('content', '')
                source = payload.get('source_document', 'Unknown')
                page = payload.get('page_number', '')
                title = payload.get('section_title', '')

                formatted_chunk = f"Source: {source}"
                if page:
                    formatted_chunk += f", Page: {page}"
                if title:
                    formatted_chunk += f", Section: {title}"
                formatted_chunk += f"\nContent: {content}\n"

                formatted_results.append(formatted_chunk)

            result = "\n".join(formatted_results)

            if not result:
                result = "No relevant content found in the knowledge base."

            self.logger.info(f"Retrieved {len(chunks)} chunks for query: {query[:50]}...")
            return result

        except Exception as e:
            self.logger.error(f"Error in retrieve_content_tool: {e}")
            return f"Error retrieving content: {str(e)}"