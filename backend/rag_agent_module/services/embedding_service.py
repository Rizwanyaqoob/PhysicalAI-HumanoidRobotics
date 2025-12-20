"""
Embedding service for the RAG Agent Backend using Cohere
"""
import cohere
import logging
from typing import List
from ..config.settings import settings
from ..config.logger import logger


class EmbeddingService:
    """
    Service class for generating embeddings using Cohere
    """
    def __init__(self):
        self.cohere_client = cohere.Client(settings.COHERE_API_KEY)
        self.logger = logger

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere
        """
        try:
            # Cohere has a limit on the number of texts per request
            batch_size = 96  # Safe limit below Cohere's max
            all_embeddings = []

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]
                response = self.cohere_client.embed(
                    texts=batch,
                    model="embed-english-v3.0",  # Latest model as of 2024
                    input_type="search_query" if len(batch) == 1 else "search_document"
                )
                all_embeddings.extend(response.embeddings)

            self.logger.info(f"Generated embeddings for {len(all_embeddings)} text chunks")
            return all_embeddings

        except Exception as e:
            self.logger.error(f"Error generating embeddings: {e}")
            raise

    def generate_query_embedding(self, query: str) -> List[float]:
        """
        Generate embedding for a single query string
        """
        try:
            response = self.cohere_client.embed(
                texts=[query],
                model="embed-english-v3.0",
                input_type="search_query"
            )
            embedding = response.embeddings[0]
            self.logger.info(f"Generated embedding for query: {query[:50]}...")
            return embedding
        except Exception as e:
            self.logger.error(f"Error generating query embedding: {e}")
            raise

    def generate_document_embeddings(self, documents: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of documents
        """
        try:
            response = self.cohere_client.embed(
                texts=documents,
                model="embed-english-v3.0",
                input_type="search_document"
            )
            embeddings = response.embeddings
            self.logger.info(f"Generated embeddings for {len(documents)} documents")
            return embeddings
        except Exception as e:
            self.logger.error(f"Error generating document embeddings: {e}")
            raise