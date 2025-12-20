"""
Configuration settings for the RAG Agent Backend
"""
from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables
    """
    # API Keys
    GEMINI_API_KEY: Optional[str] = None
    COHERE_API_KEY: str

    # Qdrant Configuration
    QDRANT_URL: str
    QDRANT_API_KEY: Optional[str] = None
    QDRANT_COLLECTION_NAME: str = "book_content"

    # Application Configuration
    LOG_LEVEL: str = "INFO"
    HOST: str = "0.0.0.0"
    PORT: int = 8000

    class Config:
        env_file = ".env"
        case_sensitive = False  # Pydantic will handle case conversion
        extra = "allow"  # Allow extra environment variables


# Create a single instance of settings
settings = Settings()