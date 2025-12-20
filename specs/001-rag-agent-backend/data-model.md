# Data Model: RAG Agent Backend

## Entities

### Query
- **Description**: A user's question or request for information that requires RAG processing
- **Fields**:
  - `query_text` (string): The text of the user's query
  - `user_id` (string, optional): Identifier for the user making the query
  - `max_chunks` (int, default: 5): Maximum number of chunks to retrieve
  - `provider` (string, enum: ['openai', 'gemini'], default: 'openai'): AI provider to use

### RetrievedContext
- **Description**: Relevant book content chunks retrieved from Qdrant based on semantic similarity to the query
- **Fields**:
  - `chunks` (list[Chunk]): List of retrieved content chunks
  - `retrieval_time` (float): Time taken for retrieval in seconds
  - `retrieval_metadata` (dict): Additional metadata about the retrieval process

### Chunk
- **Description**: A segment of book content retrieved from the vector database
- **Fields**:
  - `content` (string): The actual text content of the chunk
  - `source_document` (string): Reference to the original document
  - `page_number` (int, optional): Page number in the original document
  - `section_title` (string, optional): Title of the section containing the chunk
  - `similarity_score` (float): Cosine similarity score to the query
  - `chunk_id` (string): Unique identifier for the chunk

### GeneratedAnswer
- **Description**: The final response created by the AI agent using the retrieved context
- **Fields**:
  - `answer_text` (string): The generated answer text
  - `sources` (list[Chunk]): List of chunks used to generate the answer
  - `provider_used` (string): Which AI provider was used ('openai' or 'gemini')
  - `generation_time` (float): Time taken for answer generation in seconds
  - `debug_info` (dict): Debugging information including timing and retrieval details

### APIConfiguration
- **Description**: Settings and credentials for OpenAI and Gemini API access
- **Fields**:
  - `openai_api_key` (string): API key for OpenAI services
  - `gemini_api_key` (string): API key for Gemini services
  - `cohere_api_key` (string): API key for Cohere embedding services
  - `qdrant_url` (string): URL for Qdrant vector database
  - `qdrant_api_key` (string, optional): API key for Qdrant (if required)
  - `qdrant_collection_name` (string): Name of the collection in Qdrant to query

## State Transitions

### Query Processing Flow
1. **Query Received**: User submits a query to the API
2. **Embedding Generated**: Query text is converted to vector embedding using Cohere
3. **Retrieval Executed**: Qdrant is queried for relevant chunks based on embedding
4. **Answer Generated**: AI agent generates response using retrieved context
5. **Response Returned**: Structured response with answer and metadata is returned to user

## Validation Rules

### Query Validation
- `query_text` must not be empty
- `query_text` length must be less than 10000 characters
- `max_chunks` must be between 1 and 20
- `provider` must be either 'openai' or 'gemini'

### Chunk Validation
- `content` must not be empty
- `similarity_score` must be between 0 and 1
- `chunk_id` must be unique

### API Configuration Validation
- API keys must be provided for the selected provider
- URLs must be valid