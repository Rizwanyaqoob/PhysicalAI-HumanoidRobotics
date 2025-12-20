"""
RAG Agent Server with proper environment variable support
This script starts the FastAPI server with the port specified in the .env file
"""

import os
from dotenv import load_dotenv
import uvicorn
from rag_api import app

# Load environment variables
load_dotenv()

def main():
    # Get port from environment variable, default to 8009
    port = int(os.getenv('PORT', 8009))
    host = os.getenv('HOST', '127.0.0.1')

    print(f"Starting RAG Agent server on {host}:{port}")
    print(f"Backend API URL: http://{host}:{port}")

    uvicorn.run(
        "rag_api:app",
        host=host,
        port=port,
        reload=True,  # Enable auto-reload for development
        log_level="info"
    )

if __name__ == "__main__":
    main()