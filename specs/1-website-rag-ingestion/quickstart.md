# Quickstart: Website Content Extraction and RAG Ingestion Pipeline

## Prerequisites

- Python 3.11 or higher
- UV package manager
- Access to Cohere API (API key)
- Access to Qdrant Cloud (API key and URL)

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Create backend directory and initialize project**
   ```bash
   mkdir backend
   cd backend
   ```

3. **Set up Python environment with UV**
   ```bash
   # Install UV if not already installed
   pip install uv

   # Create virtual environment
   uv venv

   # Activate the environment
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

4. **Install required dependencies**
   ```bash
   uv pip install requests trafilatura beautifulsoup4 cohere qdrant-client python-dotenv
   ```

5. **Create environment file**
   ```bash
   touch .env
   ```

6. **Configure environment variables in `.env`**
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_URL=your_qdrant_cloud_url_here
   TARGET_SITE_URL=https://rizwanyaqoob.github.io/PhysicalAI-HumanoidRobotics/
   ```

## Usage

1. **Run the ingestion pipeline**
   ```bash
   python main.py
   ```

## Expected Output

The pipeline will:
1. Crawl all pages from the target Docusaurus site
2. Extract and clean text content from each page
3. Chunk the content into 500-1000 token segments
4. Generate embeddings using Cohere
5. Store the vectors in Qdrant Cloud with metadata
6. Log the progress and final summary

## Verification

After running the pipeline, verify:
- All pages were crawled successfully
- Text extraction preserved semantic structure
- Chunks are within the expected token range
- Embeddings were generated for all chunks
- Vectors are stored in Qdrant Cloud with proper metadata