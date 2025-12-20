import requests
import json

# Test the backend API to see if it retrieves real content
def test_real_content():
    base_url = "http://localhost:8009"

    print("Testing if RAG Agent retrieves real content...")

    # Test /api/retrieve endpoint to see if it returns real content from the ingested pages
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
                    print("  [NOTE] This appears to be mock content")
                else:
                    print("  [SUCCESS] This appears to be real content from your ingested pages!")
            else:
                print("  [WARNING] No chunks returned")
        else:
            print(f"[ERROR] /api/retrieve failed with status {response.status_code}")
            print(f"  Response: {response.text}")
    except Exception as e:
        print(f"[ERROR] Error testing /api/retrieve: {e}")

if __name__ == "__main__":
    test_real_content()