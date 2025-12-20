"""
Logging configuration for the RAG Agent Backend
"""
import logging
import sys
from typing import Optional
from .settings import settings


def setup_logger(name: str, level: Optional[str] = None) -> logging.Logger:
    """
    Set up a logger with the specified name and level
    """
    if level is None:
        level = settings.LOG_LEVEL

    logger = logging.getLogger(name)
    logger.setLevel(getattr(logging, level.upper()))

    # Avoid adding multiple handlers to the same logger
    if not logger.handlers:
        handler = logging.StreamHandler(sys.stdout)
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    return logger


# Create a root logger for the application
logger = setup_logger(__name__, settings.LOG_LEVEL)