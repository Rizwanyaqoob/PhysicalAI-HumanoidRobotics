# Data Model: RAG Retrieval Pipeline Validation

## Core Entities

### QdrantCollection
Represents the vector database collection containing embeddings with metadata
- **name** (string): The name of the collection (e.g., "RAG_embeddings")
- **vector_count** (integer): Total number of vectors in the collection
- **vector_dimensions** (integer): Dimension size of each vector (expected: 1024 for Cohere)
- **schema** (object): Structure definition of metadata fields
- **health_status** (enum): Status of the collection (healthy, degraded, unavailable)

### SearchQuery
Represents a user query that will be used to test retrieval functionality
- **id** (string): Unique identifier for the query
- **text** (string): The actual query text
- **type** (enum): Query type (broad, specific, technical)
- **expected_topic** (string): Topic area the query should match
- **created_timestamp** (datetime): When the query was created

### RetrievedChunk
Represents a text chunk returned by the semantic search with associated metadata
- **id** (string): Unique identifier for the chunk in Qdrant
- **content** (string): The actual text content of the chunk
- **similarity_score** (float): Cosine similarity score from the search
- **metadata** (object): Associated metadata (URL, title, content preview, etc.)
- **relevance_score** (float): Manual or automated relevance assessment

### DiagnosticReport
Contains metrics about retrieval quality, performance, and system health
- **validation_timestamp** (datetime): When the validation was run
- **total_queries_run** (integer): Number of queries executed during validation
- **avg_retrieval_latency** (float): Average time for retrieval in milliseconds
- **metadata_integrity_percentage** (float): Percentage of chunks with complete metadata
- **relevance_accuracy** (float): Accuracy of retrieved chunks for test queries
- **collection_health_status** (string): Overall health status of the collection
- **missing_vectors_count** (integer): Number of missing or empty vectors found
- **performance_metrics** (object): Detailed performance measurements
- **quality_metrics** (object): Detailed quality measurements
- **issues_found** (array[object]): List of any issues discovered during validation

## Relationships

1. **QdrantCollection** → **RetrievedChunk** (one-to-many)
   - One collection contains many retrieved chunks

2. **SearchQuery** → **RetrievedChunk** (one-to-many)
   - One query can return multiple chunks (top-k results)

3. **DiagnosticReport** → **SearchQuery** (one-to-many)
   - One report summarizes results from multiple queries

## Validation Rules

1. **QdrantCollection.vector_count** must match expected ingestion count (35 vectors from previous run)
2. **RetrievedChunk.metadata** must contain required fields: url, title, content_preview
3. **DiagnosticReport.metadata_integrity_percentage** must be ≥ 99%
4. **DiagnosticReport.avg_retrieval_latency** must be < 200ms
5. **DiagnosticReport.missing_vectors_count** must be 0

## State Transitions

1. **DiagnosticReport**:
   - Initial → In Progress (when validation starts)
   - In Progress → Complete (when validation finishes)
   - Complete → Published (when report is finalized)