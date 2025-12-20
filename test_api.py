import requests
import json

# Test the backend API
def test_backend():
    base_url = "http://localhost:8009"

    print("Testing RAG Agent API...")

    # Test /api/ask endpoint
    try:
        response = requests.post(f"{base_url}/api/ask",
                               json={
                                   "query": "What is humanoid robotics?",
                                   "provider": "openai",
                                   "max_chunks": 3
                               })
        if response.status_code == 200:
            data = response.json()
            print("[OK] /api/ask endpoint working")
            print(f"  Answer: {data['answer'][:100]}...")
            print(f"  Sources: {len(data['sources'])} chunks")
            print(f"  Provider: {data['provider_used']}")
        else:
            print(f"[ERROR] /api/ask failed with status {response.status_code}")
            print(f"  Response: {response.text}")
    except Exception as e:
        print(f"[ERROR] Error testing /api/ask: {e}")

    # Test /health endpoint
    try:
        response = requests.get(f"{base_url}/health")
        if response.status_code == 200:
            print("[OK] /health endpoint working")
        else:
            print(f"[ERROR] /health failed with status {response.status_code}")
    except Exception as e:
        print(f"[ERROR] Error testing /health: {e}")

    # Test /api/retrieve endpoint
    try:
        response = requests.post(f"{base_url}/api/retrieve",
                               json={
                                   "query": "humanoid robotics",
                                   "max_chunks": 3
                               })
        if response.status_code == 200:
            data = response.json()
            print("[OK] /api/retrieve endpoint working")
            print(f"  Retrieved: {len(data['chunks'])} chunks")
        else:
            print(f"[ERROR] /api/retrieve failed with status {response.status_code}")
            print(f"  Response: {response.text}")
    except Exception as e:
        print(f"[ERROR] Error testing /api/retrieve: {e}")

if __name__ == "__main__":
    test_backend()