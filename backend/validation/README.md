# RAG Retrieval Pipeline Validation

This directory contains the validation pipeline for the RAG (Retrieval-Augmented Generation) system. The pipeline validates the retrieval functionality by connecting to Qdrant Cloud, running semantic search tests, validating metadata integrity, benchmarking performance, and generating diagnostic reports.

## Overview

The RAG retrieval validation pipeline performs the following checks:

1. **Collection Health**: Verifies the Qdrant collection exists and is accessible
2. **Semantic Search**: Tests retrieval quality with various query types
3. **Metadata Integrity**: Validates that all chunks have correct metadata
4. **Performance**: Benchmarks retrieval latency and throughput
5. **Data Quality**: Checks for missing vectors, empty payloads, and schema issues
6. **Load Testing**: Validates behavior under high query load conditions

## Prerequisites

- Python 3.11+
- Qdrant Cloud account and collection
- Cohere API key
- Access to the RAG_embeddings collection

## Installation

1. Install the required dependencies:

```bash
pip install -r requirements-validation.txt
```

2. Set up your environment variables by creating a `.env` file in the backend directory:

```bash
# Required
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_url_here

# Optional - defaults shown
QDRANT_COLLECTION_NAME=RAG_embeddings
TEST_QUERY_K=10
LATENCY_TARGET_MS=200.0
METADATA_INTEGRITY_TARGET=99.0
RELEVANCE_ACCURACY_TARGET=90.0
EXPECTED_VECTOR_COUNT=35
COHERE_MODEL=embed-english-v3.0
SAMPLE_SIZE_FOR_DATA_QUALITY=100
MAX_QUERY_SAMPLES=20
QDRANT_TIMEOUT=30
COHERE_TIMEOUT=30
HTTP_TIMEOUT=30
```

## Usage

### Running the Validation Pipeline

To run the complete validation pipeline:

```bash
cd backend
python -m validation.retrieval_validator
```

Or from the validation directory:

```bash
python retrieval_validator.py
```

### Configuration Options

The validation pipeline can be configured through environment variables or by passing a configuration dictionary to the `RetrievalValidator` constructor:

```python
config = {
    'collection_name': 'my_collection',
    'test_query_k': 5,
    'latency_target_ms': 150.0,
    'metadata_integrity_target': 99.5,
    'relevance_accuracy_target': 95.0,
    'expected_vector_count': 100,
    'cohere_model': 'embed-english-v3.0',
    'sample_size_for_data_quality': 50,
    'max_query_samples': 10,
    'qdrant_timeout': 60,
    'cohere_timeout': 60,
    'http_timeout': 60
}

validator = RetrievalValidator(config=config)
results = validator.run_validation()
```

### Using as a Context Manager

For automatic resource cleanup:

```python
from validation.retrieval_validator import RetrievalValidator

with RetrievalValidator() as validator:
    results = validator.run_validation()
    report = validator.generate_report()
```

## Test Queries

The validation pipeline uses test queries loaded from `test_queries.json`. You can customize these queries to test specific topics relevant to your RAG system. The default queries cover various robotics and AI topics:

- Embodied intelligence
- Robot control systems
- Reinforcement learning
- Physical AI
- Motion planning
- Computer vision in robotics
- VLA systems
- ROS2 in robotics
- Digital twin technology
- Testing and debugging

## Output

The validation pipeline generates:

1. **JSON Reports**: Detailed validation results in `validation_report.json` and `validation_report_raw.json`
2. **Historical Data**: Previous validation results are stored in the `validation_history` directory
3. **Console Output**: Summary of validation results printed to the console
4. **Log Files**: Detailed logging information

## Validation Metrics

The pipeline measures and reports on:

- **Collection Health**: Vector count, dimensions, and schema verification
- **Performance**: Average, min, and max retrieval latency
- **Metadata Integrity**: Percentage of chunks with complete metadata
- **Relevance Accuracy**: How well search results match query topics
- **Data Quality**: Issues like missing vectors, empty payloads, duplicates
- **Load Performance**: Throughput and response times under concurrent queries

## Issue Classification

Issues found during validation are classified by severity:

- **High**: Critical issues affecting core functionality
- **Medium**: Issues that may impact performance or reliability
- **Low**: Minor issues that don't affect core functionality

## Environment Variables

### Required Variables
- `COHERE_API_KEY`: Cohere API key for embedding queries
- `QDRANT_API_KEY`: Qdrant API key for accessing the vector database
- `QDRANT_URL`: URL for the Qdrant Cloud instance

### Optional Variables
- `QDRANT_COLLECTION_NAME`: Name of the collection to validate (default: RAG_embeddings)
- `TEST_QUERY_K`: Number of results to retrieve per query (default: 10)
- `LATENCY_TARGET_MS`: Target maximum retrieval latency in milliseconds (default: 200.0)
- `METADATA_INTEGRITY_TARGET`: Target percentage for metadata completeness (default: 99.0)
- `RELEVANCE_ACCURACY_TARGET`: Target percentage for relevance accuracy (default: 90.0)
- `EXPECTED_VECTOR_COUNT`: Expected number of vectors in the collection (default: 35)

## Troubleshooting

### Common Issues

1. **Connection Errors**: Verify QDRANT_URL and QDRANT_API_KEY are correct
2. **Authentication Errors**: Check API keys for COHERE_API_KEY and QDRANT_API_KEY
3. **Collection Not Found**: Ensure QDRANT_COLLECTION_NAME matches an existing collection
4. **High Latency**: May indicate rate limiting or network issues
5. **Low Relevance**: May indicate issues with the embedding model or indexing process

### Logging

The validation pipeline uses comprehensive logging. Check the logs for detailed information about validation steps and any issues encountered.

## Integration with CI/CD

The validation pipeline can be integrated into CI/CD workflows to ensure the RAG system remains healthy:

```bash
# Run validation and check results
python -m validation.retrieval_validator

# Check exit code or parse results to determine if validation passed
```

## Performance Considerations

- The validation pipeline may take several minutes to complete depending on the size of your collection and network conditions
- High load testing may impact the performance of your Qdrant instance temporarily
- Consider running validation during off-peak hours to minimize impact on production workloads

## Security

- Store API keys securely and never commit them to version control
- Use environment variables or secure key management systems
- The validation pipeline does not modify data in Qdrant, only reads from it