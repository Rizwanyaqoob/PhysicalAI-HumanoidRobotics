"""
Health check service for the RAG Agent Backend
"""
from typing import Dict, Any
from enum import Enum
from datetime import datetime
from ..config.settings import settings
from ..config.logger import logger
from .embedding_service import EmbeddingService
from .retrieval_service import RetrievalService
from .agent_service import AgentService


class HealthStatus(Enum):
    """
    Enum for health status
    """
    HEALTHY = "healthy"
    UNHEALTHY = "unhealthy"


class DependencyStatus(Enum):
    """
    Enum for dependency status
    """
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"


class HealthService:
    """
    Service class for health checking the RAG Agent Backend and its dependencies
    """
    def __init__(self):
        self.logger = logger
        self.embedding_service = EmbeddingService()
        self.retrieval_service = RetrievalService()
        self.agent_service = AgentService()

    def get_health_status(self) -> Dict[str, Any]:
        """
        Get the overall health status of the service and its dependencies
        """
        timestamp = datetime.now().isoformat()

        # Check dependencies
        dependencies = {
            "qdrant": self._check_qdrant_connection(),
            "openai": self._check_openai_connection(),
            "cohere": self._check_cohere_connection(),
            "gemini": self._check_gemini_connection()
        }

        # Determine overall status
        all_connected = all(status == DependencyStatus.CONNECTED for status in dependencies.values())
        overall_status = HealthStatus.HEALTHY if all_connected else HealthStatus.UNHEALTHY

        health_info = {
            "status": overall_status.value,
            "timestamp": timestamp,
            "dependencies": {k: v.value for k, v in dependencies.items()}
        }

        if not all_connected:
            health_info["error"] = "One or more dependencies are unavailable"

        return health_info

    def _check_qdrant_connection(self) -> DependencyStatus:
        """
        Check if Qdrant connection is working
        """
        try:
            # Test if collection exists
            collection_exists = self.retrieval_service.verify_collection_exists()
            if collection_exists:
                return DependencyStatus.CONNECTED
            else:
                self.logger.warning("Qdrant collection does not exist")
                return DependencyStatus.DISCONNECTED
        except Exception as e:
            self.logger.error(f"Qdrant connection check failed: {e}")
            return DependencyStatus.DISCONNECTED

    def _check_openai_connection(self) -> DependencyStatus:
        """
        Check if OpenAI connection is working
        """
        try:
            return DependencyStatus.CONNECTED if self.agent_service.test_openai_connection() else DependencyStatus.DISCONNECTED
        except Exception as e:
            self.logger.error(f"OpenAI connection check failed: {e}")
            return DependencyStatus.DISCONNECTED

    def _check_gemini_connection(self) -> DependencyStatus:
        """
        Check if Gemini connection is working
        """
        try:
            return DependencyStatus.CONNECTED if self.agent_service.test_gemini_connection() else DependencyStatus.DISCONNECTED
        except Exception as e:
            self.logger.error(f"Gemini connection check failed: {e}")
            return DependencyStatus.DISCONNECTED

    def _check_cohere_connection(self) -> DependencyStatus:
        """
        Check if Cohere connection is working by generating a simple embedding
        """
        try:
            # Test with a simple embedding
            self.embedding_service.generate_query_embedding("test")
            return DependencyStatus.CONNECTED
        except Exception as e:
            self.logger.error(f"Cohere connection check failed: {e}")
            return DependencyStatus.DISCONNECTED

    def ping(self) -> Dict[str, str]:
        """
        Simple ping endpoint to check if the service is running
        """
        return {"status": "alive", "timestamp": datetime.now().isoformat()}