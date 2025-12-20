# Quickstart: RAG Retrieval Pipeline Validation

## Prerequisites

- Python 3.11 or higher
- Access to Qdrant Cloud (API key and URL from existing RAG pipeline)
- Access to Cohere API (API key from existing RAG pipeline)
- Existing RAG_embeddings collection in Qdrant Cloud with data

## Setup

1. **Navigate to backend directory**
   ```bash
   cd backend
   ```

2. **Install validation dependencies**
   ```bash
   pip install qdrant-client cohere python-dotenv requests
   ```

3. **Configure environment variables**
   Ensure your `.env` file contains:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   ```

4. **Verify existing RAG pipeline data**
   Make sure the RAG_embeddings collection exists in Qdrant Cloud with the expected data from the ingestion pipeline.

## Usage

1. **Run the validation pipeline**
   ```bash
   python validation/retrieval_validator.py
   ```

2. **View the validation report**
   The validation will generate a diagnostic report in JSON format and a human-readable summary.

## Expected Output

The validation pipeline will:
1. Connect to Qdrant Cloud and verify collection health
2. Run semantic search tests with sample queries
3. Validate metadata integrity across all chunks
4. Benchmark retrieval performance
5. Generate a diagnostic report with quality metrics
6. Identify any missing or corrupted vectors

## Verification

After running the validation, verify:
- Collection exists and has correct schema
- Retrieval latency is under 200ms
- Metadata integrity is ≥ 99%
- All 35 ingested chunks are accessible
- Semantic search returns relevant results
- Diagnostic report shows no critical issues