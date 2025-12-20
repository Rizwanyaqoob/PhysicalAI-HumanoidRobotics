"""
Simple test/demo script for the RAG Agent Backend
"""
from rag_agent.services.embedding_service import EmbeddingService
from rag_agent.services.retrieval_service import RetrievalService
from rag_agent.services.agent_service import AgentService
from rag_agent.config.settings import settings
from rag_agent.config.logger import logger


def test_connections():
    """
    Test connections to all external services
    """
    print("Testing connections to external services...")

    # Test embedding service
    try:
        embedding_service = EmbeddingService()
        test_embedding = embedding_service.generate_query_embedding("test")
        print(f"OK Cohere embedding service: OK (embedding length: {len(test_embedding)})")
    except Exception as e:
        print(f"ERROR Cohere embedding service: ERROR - {e}")

    # Test retrieval service
    try:
        retrieval_service = RetrievalService()
        collection_exists = retrieval_service.verify_collection_exists()
        print(f"OK Qdrant service: OK (collection exists: {collection_exists})")
    except Exception as e:
        print(f"ERROR Qdrant service: ERROR - {e}")

    # Test agent service
    if settings.OPENAI_API_KEY or settings.GEMINI_API_KEY:
        try:
            agent_service = AgentService()
            openai_ok = agent_service.test_openai_connection() if settings.OPENAI_API_KEY else False
            gemini_ok = agent_service.test_gemini_connection() if settings.GEMINI_API_KEY else False
            print(f"OK OpenAI agent service: {'OK' if openai_ok else 'Not configured'}")
            print(f"OK Gemini agent service: {'OK' if gemini_ok else 'Not configured'}")
        except Exception as e:
            print(f"ERROR Agent services: ERROR - {e}")
    else:
        print("INFO No API keys configured, skipping agent service test")
        print("OK OpenAI agent service: Not configured")
        print("OK Gemini agent service: Not configured")


def demo_rag_flow():
    """
    Demonstrate a simplified RAG flow
    """
    print("\nDemonstrating RAG flow...")

    try:
        # Initialize services
        embedding_service = EmbeddingService()
        retrieval_service = RetrievalService()

        # Sample query
        query = "What are the basic principles of robotics?"

        print(f"Query: {query}")

        # Step 1: Generate embedding
        query_embedding = embedding_service.generate_query_embedding(query)
        print(f"OK Generated embedding with {len(query_embedding)} dimensions")

        # Step 2: Retrieve relevant chunks (this will fail if no data in Qdrant)
        try:
            chunks = retrieval_service.retrieve_chunks(query_embedding, max_chunks=3)
            print(f"OK Retrieved {len(chunks)} chunks from Qdrant")

            # If we have chunks, format them for the agent
            if chunks:
                from rag_agent.utils.helpers import format_chunks_for_agent
                formatted_chunks = format_chunks_for_agent(chunks)

                # Initialize agent service only if we have chunks and API keys
                if settings.OPENAI_API_KEY or settings.GEMINI_API_KEY:
                    agent_service = AgentService()

                    # Step 3: Generate answer using OpenAI (if available)
                    if settings.OPENAI_API_KEY:
                        answer = agent_service.generate_answer_with_openai(query, formatted_chunks)
                        print(f"OK Generated answer using OpenAI")
                        print(f"Answer preview: {answer[:100]}...")
                    elif settings.GEMINI_API_KEY:
                        answer = agent_service.generate_answer_with_gemini(query, formatted_chunks)
                        print(f"OK Generated answer using Gemini")
                        print(f"Answer preview: {answer[:100]}...")
                    else:
                        print("INFO No API keys configured, skipping answer generation")
                else:
                    print("INFO No API keys configured, skipping answer generation")

        except Exception as e:
            print(f"INFO No content retrieved from Qdrant (expected if collection is empty): {e}")

    except Exception as e:
        print(f"ERROR RAG flow demo failed: {e}")


if __name__ == "__main__":
    print("RAG Agent Backend - Test & Demo Script")
    print("=" * 50)

    test_connections()
    demo_rag_flow()

    print("\n" + "=" * 50)
    print("Test & Demo completed!")