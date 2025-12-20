import requests
import json

# Test the backend API to see if it retrieves real content specifically using Gemini
def test_gemini_directly():
    base_url = "http://localhost:8009"

    print("Testing RAG Agent API with Gemini provider...")

    # Test /api/ask endpoint specifically with gemini provider
    try:
        response = requests.post(f"{base_url}/api/ask",
                               json={
                                   "query": "What is humanoid robotics?",
                                   "provider": "gemini",  # Specifically request Gemini
                                   "max_chunks": 3
                               })
        if response.status_code == 200:
            data = response.json()
            print(f"[OK] /api/ask endpoint working with Gemini")
            print(f"  Answer: {data['answer'][:100]}...")
            print(f"  Sources: {len(data['sources'])} chunks")
            print(f"  Provider: {data['provider_used']}")

            # Check if sources are real
            if len(data['sources']) > 0:
                first_source = data['sources'][0]
                print(f"  First source: {first_source.get('source_document', 'Unknown')}")
                if "sample_document" in first_source.get('source_document', ''):
                    print("  [ISSUE] Still getting mock content")
                else:
                    print("  [SUCCESS] Getting real content from Qdrant!")
        else:
            print(f"[ERROR] /api/ask failed with status {response.status_code}")
            print(f"  Response: {response.text}")
    except Exception as e:
        print(f"[ERROR] Error testing /api/ask: {e}")

    # Also test retrieve to see what's in the database
    print("\nTesting /api/retrieve...")
    try:
        response = requests.post(f"{base_url}/api/retrieve",
                               json={
                                   "query": "humanoid robotics",
                                   "max_chunks": 3
                               })
        if response.status_code == 200:
            data = response.json()
            print(f"[OK] /api/retrieve endpoint working")
            print(f"  Retrieved: {len(data['chunks'])} chunks")
            if len(data['chunks']) > 0:
                first_chunk = data['chunks'][0]
                print(f"  First chunk source: {first_chunk.get('source_document', 'Unknown')}")
                print(f"  First chunk preview: {first_chunk.get('content', '')[:100]}...")
                if "sample_document" in first_chunk.get('source_document', ''):
                    print("  [ISSUE] Retrieve is also returning mock content")
                else:
                    print("  [INTERESTING] Retrieve has real content but Ask doesn't")
        else:
            print(f"[ERROR] /api/retrieve failed with status {response.status_code}")
            print(f"  Response: {response.text}")
    except Exception as e:
        print(f"[ERROR] Error testing /api/retrieve: {e}")

if __name__ == "__main__":
    test_gemini_directly()