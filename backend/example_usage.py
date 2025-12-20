"""
Example usage scripts for the RAG Agent Backend
"""
import asyncio
import httpx
import json


async def example_ask_query():
    """
    Example of how to use the /api/ask endpoint
    """
    async with httpx.AsyncClient(timeout=30.0) as client:
        # Example query using OpenAI
        data = {
            "query": "What are the key principles of humanoid robotics?",
            "provider": "openai",
            "max_chunks": 5,
            "user_id": "example-user"
        }

        try:
            response = await client.post("http://localhost:8000/api/ask", json=data)
            response.raise_for_status()
            result = response.json()

            print("Ask Endpoint Response:")
            print(json.dumps(result, indent=2))
            print("\nAnswer:", result.get("answer", "No answer returned"))
            print(f"Sources: {len(result.get('sources', []))} chunks used")
            print(f"Provider used: {result.get('provider_used')}")
            print(f"Total time: {result.get('debug_info', {}).get('total_time', 0):.2f}s")

        except httpx.RequestError as e:
            print(f"Request error: {e}")
        except httpx.HTTPStatusError as e:
            print(f"HTTP error: {e}")
            print(f"Response: {e.response.text}")


async def example_retrieve_query():
    """
    Example of how to use the /api/retrieve endpoint
    """
    async with httpx.AsyncClient(timeout=30.0) as client:
        # Example retrieval query
        data = {
            "query": "What are the key principles of humanoid robotics?",
            "max_chunks": 3
        }

        try:
            response = await client.post("http://localhost:8000/api/retrieve", json=data)
            response.raise_for_status()
            result = response.json()

            print("\nRetrieve Endpoint Response:")
            print(json.dumps(result, indent=2))
            print(f"Retrieved {len(result.get('chunks', []))} chunks")
            print(f"Retrieval time: {result.get('retrieval_time', 0):.2f}s")

            # Print first chunk as example
            if result.get('chunks'):
                first_chunk = result['chunks'][0]
                print(f"First chunk content preview: {first_chunk.get('content', '')[:100]}...")

        except httpx.RequestError as e:
            print(f"Request error: {e}")
        except httpx.HTTPStatusError as e:
            print(f"HTTP error: {e}")
            print(f"Response: {e.response.text}")


async def example_health_check():
    """
    Example of how to use the /health endpoint
    """
    async with httpx.AsyncClient(timeout=30.0) as client:
        try:
            response = await client.get("http://localhost:8000/health")
            response.raise_for_status()
            result = response.json()

            print("\nHealth Check Response:")
            print(json.dumps(result, indent=2))

        except httpx.RequestError as e:
            print(f"Request error: {e}")
        except httpx.HTTPStatusError as e:
            print(f"HTTP error: {e}")
            print(f"Response: {e.response.text}")


async def example_compare_providers():
    """
    Example comparing OpenAI and Gemini responses for the same query
    """
    async with httpx.AsyncClient(timeout=30.0) as client:
        query = "What are the ethical considerations in robotics?"
        providers = ["openai", "gemini"]

        for provider in providers:
            data = {
                "query": query,
                "provider": provider,
                "max_chunks": 3
            }

            try:
                response = await client.post("http://localhost:8000/api/ask", json=data)
                response.raise_for_status()
                result = response.json()

                print(f"\n{provider.upper()} Response:")
                print(f"Answer length: {len(result.get('answer', ''))} characters")
                print(f"Sources: {len(result.get('sources', []))} chunks")
                print(f"Generation time: {result.get('debug_info', {}).get('generation_time', 0):.2f}s")
                print(f"Full answer preview: {result.get('answer', '')[:200]}...")

            except httpx.RequestError as e:
                print(f"Request error for {provider}: {e}")
            except httpx.HTTPStatusError as e:
                print(f"HTTP error for {provider}: {e}")
                print(f"Response: {e.response.text}")


async def main():
    """
    Run all example scripts
    """
    print("Running RAG Agent Backend example usage scripts...\n")

    print("=" * 50)
    print("1. ASK ENDPOINT EXAMPLE")
    print("=" * 50)
    await example_ask_query()

    print("\n" + "=" * 50)
    print("2. RETRIEVE ENDPOINT EXAMPLE")
    print("=" * 50)
    await example_retrieve_query()

    print("\n" + "=" * 50)
    print("3. HEALTH CHECK EXAMPLE")
    print("=" * 50)
    await example_health_check()

    print("\n" + "=" * 50)
    print("4. PROVIDER COMPARISON EXAMPLE")
    print("=" * 50)
    await example_compare_providers()

    print("\n" + "=" * 50)
    print("Examples completed!")
    print("=" * 50)


if __name__ == "__main__":
    asyncio.run(main())