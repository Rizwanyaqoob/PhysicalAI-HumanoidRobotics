import os
from dotenv import load_dotenv
load_dotenv()

# Test each service individually
print("Testing environment variables:")
print(f"GEMINI_API_KEY exists: {bool(os.getenv('GEMINI_API_KEY'))}")
print(f"COHERE_API_KEY exists: {bool(os.getenv('COHERE_API_KEY'))}")
print(f"QDRANT_URL exists: {bool(os.getenv('QDRANT_URL'))}")
print(f"QDRANT_API_KEY exists: {bool(os.getenv('QDRANT_API_KEY'))}")

# Test Qdrant connection
try:
    from qdrant_client import QdrantClient
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
    )
    # Test connection by getting collections
    collections = qdrant_client.get_collections()
    print(f"[OK] Qdrant connection successful. Collections: {[col.name for col in collections.collections]}")
except Exception as e:
    print(f"[ERROR] Qdrant connection failed: {e}")

# Test Cohere connection
try:
    import cohere
    cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
    # Test with a simple embedding
    response = cohere_client.embed(texts=["test"], model="embed-english-v3.0")
    print("[OK] Cohere connection successful")
except Exception as e:
    print(f"[ERROR] Cohere connection failed: {e}")

# Test OpenAI connection
try:
    from openai import OpenAI
    openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))  # This might be None
    if openai_client.api_key:
        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": "test"}],
            max_tokens=5
        )
        print("[OK] OpenAI connection successful")
    else:
        print("[INFO] OpenAI API key not set, but that's OK if using Gemini")
except Exception as e:
    print(f"[ERROR] OpenAI connection failed: {e}")

# Test Gemini if the key exists
try:
    import google.generativeai as genai
    gemini_key = os.getenv("GEMINI_API_KEY")
    if gemini_key:
        genai.configure(api_key=gemini_key)
        model = genai.GenerativeModel('gemini-pro')  # or gemini-1.0-pro
        print("[OK] Gemini configuration successful")
    else:
        print("[ERROR] GEMINI_API_KEY not found")
except Exception as e:
    print(f"[ERROR] Gemini connection failed: {e}")