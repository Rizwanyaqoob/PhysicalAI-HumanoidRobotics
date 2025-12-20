"""
RAG Retrieval Pipeline Validation

This script validates the retrieval functionality of the RAG pipeline by:
1. Connecting to Qdrant Cloud and verifying collection health
2. Running semantic search tests with sample queries
3. Validating metadata integrity
4. Benchmarking performance
5. Generating diagnostic reports
"""

import os
import time
import json
import uuid
from typing import List, Dict, Tuple, Any
import requests
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from dotenv import load_dotenv
import logging
from datetime import datetime

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Set up requests session with appropriate headers
session = requests.Session()
session.headers.update({
    'User-Agent': 'RAG-Validation-Pipeline/1.0',
    'Accept': 'application/json',
    'Content-Type': 'application/json'
})

class RetrievalValidator:
    def __init__(self, config=None):
        # Default configuration
        self.config = {
            'collection_name': os.getenv('QDRANT_COLLECTION_NAME', 'RAG_embeddings'),
            'test_query_k': int(os.getenv('TEST_QUERY_K', '10')),
            'latency_target_ms': float(os.getenv('LATENCY_TARGET_MS', '200.0')),
            'metadata_integrity_target': float(os.getenv('METADATA_INTEGRITY_TARGET', '99.0')),
            'relevance_accuracy_target': float(os.getenv('RELEVANCE_ACCURACY_TARGET', '90.0')),
            'expected_vector_count': int(os.getenv('EXPECTED_VECTOR_COUNT', '35')),
            'cohere_model': os.getenv('COHERE_MODEL', 'embed-english-v3.0'),
            'sample_size_for_data_quality': int(os.getenv('SAMPLE_SIZE_FOR_DATA_QUALITY', '100')),
            'max_query_samples': int(os.getenv('MAX_QUERY_SAMPLES', '20')),
            'qdrant_timeout': int(os.getenv('QDRANT_TIMEOUT', '30')),  # Default 30 seconds
            'cohere_timeout': int(os.getenv('COHERE_TIMEOUT', '30')),   # Default 30 seconds
            'http_timeout': int(os.getenv('HTTP_TIMEOUT', '30'))        # Default 30 seconds
        }

        # Override with any provided config
        if config:
            # Sanitize and validate the provided config
            sanitized_config = self._sanitize_config(config)
            self.config.update(sanitized_config)

        # Validate configuration values
        self._validate_configuration()

        # Initialize API clients with timeout handling
        self.cohere_client = cohere.Client(os.getenv('COHERE_API_KEY'))
        self.qdrant_client = QdrantClient(
            url=os.getenv('QDRANT_URL'),
            api_key=os.getenv('QDRANT_API_KEY'),
            timeout=self.config['qdrant_timeout']
        )

        # Load test queries
        self.test_queries = self._load_test_queries()

        # Initialize validation results
        self.validation_results = {
            'validation_timestamp': datetime.now().isoformat(),
            'collection_health': {},
            'retrieval_metrics': [],
            'metadata_integrity': {},
            'performance_metrics': {},
            'quality_metrics': {},
            'issues_found': []
        }

        # Store initial collection schema for comparison during validation
        self.initial_collection_schema = None

    def __enter__(self):
        """
        Context manager entry method.
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """
        Context manager exit method that ensures cleanup.
        """
        self.cleanup_resources()

    def _sanitize_config(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Sanitize configuration parameters (T069).
        """
        sanitized = {}
        for key, value in config.items():
            if key in ['collection_name', 'cohere_model']:
                # For string parameters, sanitize the input
                if isinstance(value, str):
                    sanitized[key] = self._sanitize_input(value)
                else:
                    sanitized[key] = str(value) if value is not None else ''
            elif key in ['test_query_k', 'expected_vector_count', 'sample_size_for_data_quality', 'max_query_samples', 'qdrant_timeout', 'cohere_timeout', 'http_timeout']:
                # For integer parameters, ensure they are valid integers
                try:
                    sanitized[key] = int(value) if value is not None else 0
                except (ValueError, TypeError):
                    sanitized[key] = 0  # Default to 0 if conversion fails
            elif key in ['latency_target_ms', 'metadata_integrity_target', 'relevance_accuracy_target']:
                # For float parameters, ensure they are valid floats
                try:
                    sanitized[key] = float(value) if value is not None else 0.0
                except (ValueError, TypeError):
                    sanitized[key] = 0.0  # Default to 0.0 if conversion fails
            else:
                # For other parameters, just pass through
                sanitized[key] = value
        return sanitized

    def _validate_configuration(self):
        """
        Validate configuration parameters (T072).
        """
        issues = []

        # Validate collection_name
        collection_name = self.config.get('collection_name', '')
        if not collection_name or len(collection_name) > 255:
            issues.append({
                'type': 'invalid_collection_name',
                'description': f"Collection name is invalid: {collection_name}",
                'severity': self._classify_issue_severity({
                    'type': 'invalid_collection_name',
                    'description': f"Collection name is invalid: {collection_name}"
                })
            })

        # Validate test_query_k (should be between 1 and 100)
        test_query_k = self.config.get('test_query_k', 10)
        if not (1 <= test_query_k <= 100):
            issues.append({
                'type': 'invalid_test_query_k',
                'description': f"test_query_k must be between 1 and 100, got {test_query_k}",
                'severity': self._classify_issue_severity({
                    'type': 'invalid_test_query_k',
                    'description': f"test_query_k must be between 1 and 100, got {test_query_k}"
                })
            })

        # Validate timeouts (should be positive and reasonable)
        for timeout_param in ['qdrant_timeout', 'cohere_timeout', 'http_timeout']:
            timeout_val = self.config.get(timeout_param, 30)
            if not (1 <= timeout_val <= 300):  # Between 1 second and 5 minutes
                issues.append({
                    'type': 'invalid_timeout_value',
                    'description': f"{timeout_param} must be between 1 and 300 seconds, got {timeout_val}",
                    'severity': self._classify_issue_severity({
                        'type': 'invalid_timeout_value',
                        'description': f"{timeout_param} must be between 1 and 300 seconds, got {timeout_val}"
                    })
                })

        # Add validation issues to results if this method is called during validation
        if hasattr(self, 'validation_results') and issues:
            self.validation_results['issues_found'].extend(issues)

    def validate_environment_variables(self) -> List[Dict[str, str]]:
        """
        Validate required environment variables are present and valid (T071).
        """
        required_vars = [
            'COHERE_API_KEY',
            'QDRANT_API_KEY',
            'QDRANT_URL'
        ]

        issues = []

        for var in required_vars:
            value = os.getenv(var)
            if not value:
                issue = {
                    'type': 'missing_environment_variable',
                    'description': f"Required environment variable {var} is not set",
                    'severity': self._classify_issue_severity({
                        'type': 'missing_environment_variable',
                        'description': f"Required environment variable {var} is not set"
                    })
                }
                issues.append(issue)
            elif var.endswith('_API_KEY') and len(value) < 10:  # Basic validation for API keys
                issue = {
                    'type': 'invalid_api_key',
                    'description': f"Environment variable {var} appears to contain an invalid API key (too short)",
                    'severity': self._classify_issue_severity({
                        'type': 'invalid_api_key',
                        'description': f"Environment variable {var} appears to contain an invalid API key (too short)"
                    })
                }
                issues.append(issue)
            elif var == 'QDRANT_URL' and not value.startswith(('http://', 'https://')):
                issue = {
                    'type': 'invalid_url_format',
                    'description': f"Environment variable {var} does not appear to be a valid URL",
                    'severity': self._classify_issue_severity({
                        'type': 'invalid_url_format',
                        'description': f"Environment variable {var} does not appear to be a valid URL"
                    })
                }
                issues.append(issue)

        # Validate optional but recommended environment variables
        optional_vars = [
            'QDRANT_COLLECTION_NAME',
            'TEST_QUERY_K',
            'LATENCY_TARGET_MS',
            'METADATA_INTEGRITY_TARGET',
            'RELEVANCE_ACCURACY_TARGET',
            'EXPECTED_VECTOR_COUNT',
            'COHERE_MODEL',
            'SAMPLE_SIZE_FOR_DATA_QUALITY',
            'MAX_QUERY_SAMPLES',
            'QDRANT_TIMEOUT',
            'COHERE_TIMEOUT',
            'HTTP_TIMEOUT'
        ]

        for var in optional_vars:
            value = os.getenv(var)
            if value:
                # Perform basic validation on the values
                if var in ['TEST_QUERY_K', 'EXPECTED_VECTOR_COUNT', 'SAMPLE_SIZE_FOR_DATA_QUALITY', 'MAX_QUERY_SAMPLES',
                          'QDRANT_TIMEOUT', 'COHERE_TIMEOUT', 'HTTP_TIMEOUT']:
                    try:
                        int_val = int(value)
                        if int_val <= 0:
                            issue = {
                                'type': 'invalid_environment_variable_value',
                                'description': f"Environment variable {var} must be a positive integer, got {value}",
                                'severity': self._classify_issue_severity({
                                    'type': 'invalid_environment_variable_value',
                                    'description': f"Environment variable {var} must be a positive integer, got {value}"
                                })
                            }
                            issues.append(issue)
                    except ValueError:
                        issue = {
                            'type': 'invalid_environment_variable_value',
                            'description': f"Environment variable {var} must be an integer, got {value}",
                            'severity': self._classify_issue_severity({
                                'type': 'invalid_environment_variable_value',
                                'description': f"Environment variable {var} must be an integer, got {value}"
                            })
                        }
                        issues.append(issue)
                elif var in ['LATENCY_TARGET_MS', 'METADATA_INTEGRITY_TARGET', 'RELEVANCE_ACCURACY_TARGET']:
                    try:
                        float_val = float(value)
                        if float_val < 0:
                            issue = {
                                'type': 'invalid_environment_variable_value',
                                'description': f"Environment variable {var} must be a non-negative number, got {value}",
                                'severity': self._classify_issue_severity({
                                    'type': 'invalid_environment_variable_value',
                                    'description': f"Environment variable {var} must be a non-negative number, got {value}"
                                })
                            }
                            issues.append(issue)
                    except ValueError:
                        issue = {
                            'type': 'invalid_environment_variable_value',
                            'description': f"Environment variable {var} must be a number, got {value}",
                            'severity': self._classify_issue_severity({
                                'type': 'invalid_environment_variable_value',
                                'description': f"Environment variable {var} must be a number, got {value}"
                            })
                        }
                        issues.append(issue)

        return issues

    def _load_test_queries(self) -> List[Dict[str, str]]:
        """
        Load a sample set of test queries covering general and specific topics.
        """
        # Default test queries if file doesn't exist
        default_queries = [
            {
                "id": "q1",
                "text": "What is embodied intelligence?",
                "type": "specific",
                "expected_topic": "embodied intelligence"
            },
            {
                "id": "q2",
                "text": "Explain humanoid robot control systems",
                "type": "specific",
                "expected_topic": "robot control"
            },
            {
                "id": "q3",
                "text": "How does reinforcement learning apply to robotics?",
                "type": "specific",
                "expected_topic": "reinforcement learning"
            },
            {
                "id": "q4",
                "text": "What are the foundations of physical AI?",
                "type": "broad",
                "expected_topic": "physical AI"
            },
            {
                "id": "q5",
                "text": "Describe motion planning techniques",
                "type": "specific",
                "expected_topic": "motion planning"
            },
            {
                "id": "q6",
                "text": "What is computer vision in robotics?",
                "type": "broad",
                "expected_topic": "perception"
            },
            {
                "id": "q7",
                "text": "How do VLA systems work?",
                "type": "specific",
                "expected_topic": "VLA systems"
            },
            {
                "id": "q8",
                "text": "Explain ROS2 in humanoid robotics",
                "type": "specific",
                "expected_topic": "ROS2"
            }
        ]

        # Try to load from file, fallback to defaults
        try:
            with open('validation/test_queries.json', 'r') as f:
                queries = json.load(f)
                logger.info(f"Loaded {len(queries)} test queries from file")
                return queries
        except FileNotFoundError:
            logger.info(f"Test queries file not found, using {len(default_queries)} default queries")
            return default_queries

    def connect_and_verify_collection(self, collection_name: str = None, max_retries: int = 3, retry_delay: float = 1.0) -> Dict[str, Any]:
        """
        Connect to Qdrant Cloud using env vars and confirm collection health with retry logic for temporary unavailability.
        Also captures initial schema for comparison during validation (T063) and handles network timeouts (T066).
        """
        # Use provided collection name or default from config
        collection_name = collection_name or self.config['collection_name']

        logger.info(f"Connecting to Qdrant Cloud and verifying collection: {collection_name}")

        for attempt in range(max_retries):
            try:
                # Check if collection exists
                collections = self.qdrant_client.get_collections()
                collection_exists = any(col.name == collection_name for col in collections.collections)

                if not collection_exists:
                    error_msg = f"Collection {collection_name} does not exist"
                    logger.error(error_msg)
                    issue = {
                        'type': 'collection_missing',
                        'description': error_msg,
                        'severity': self._classify_issue_severity({
                            'type': 'collection_missing',
                            'description': error_msg
                        })
                    }
                    self.validation_results['issues_found'].append(issue)
                    return {'status': 'error', 'message': error_msg}

                # Get collection info
                collection_info = self.qdrant_client.get_collection(collection_name)

                # Verify schema and vector count
                vector_count = collection_info.points_count
                vector_config = collection_info.config.params.vectors

                health_info = {
                    'collection_exists': True,
                    'vector_count': vector_count,
                    'vector_dimensions': vector_config.size if hasattr(vector_config, 'size') else 'unknown',
                    'distance_type': vector_config.distance if hasattr(vector_config, 'distance') else 'unknown',
                    'status': 'healthy'
                }

                logger.info(f"Collection {collection_name} verified: {vector_count} vectors, {health_info['vector_dimensions']} dimensions")

                # Capture initial schema for comparison during validation (T063)
                self.initial_collection_schema = self._capture_collection_schema()

                return health_info

            except requests.exceptions.Timeout:
                error_msg = f"Timeout connecting to Qdrant: Request timed out after {self.config['qdrant_timeout']} seconds"
                logger.warning(f"Attempt {attempt + 1} failed: {error_msg}")

                if attempt < max_retries - 1:
                    # Wait before retrying
                    time.sleep(retry_delay)
                    # Update delay for next retry (exponential backoff)
                    retry_delay *= 2
                else:
                    logger.error(f"All {max_retries} attempts failed due to timeout. Last error: {error_msg}")
                    issue = {
                        'type': 'connection_timeout',
                        'description': error_msg,
                    }
                    issue['severity'] = self._classify_issue_severity(issue)
                    self.validation_results['issues_found'].append(issue)
                    return {'status': 'error', 'message': error_msg}
            except requests.exceptions.ConnectionError:
                error_msg = "Connection error connecting to Qdrant: Unable to connect to Qdrant API"
                logger.warning(f"Attempt {attempt + 1} failed: {error_msg}")

                if attempt < max_retries - 1:
                    # Wait before retrying
                    time.sleep(retry_delay)
                    # Update delay for next retry (exponential backoff)
                    retry_delay *= 2
                else:
                    logger.error(f"All {max_retries} attempts failed due to connection error. Last error: {error_msg}")
                    issue = {
                        'type': 'connection_error',
                        'description': error_msg,
                    }
                    issue['severity'] = self._classify_issue_severity(issue)
                    self.validation_results['issues_found'].append(issue)
                    return {'status': 'error', 'message': error_msg}
            except Exception as e:
                error_msg = f"Error connecting to Qdrant: {str(e)}"
                logger.warning(f"Attempt {attempt + 1} failed: {error_msg}")

                if attempt < max_retries - 1:
                    # Wait before retrying
                    time.sleep(retry_delay)
                    # Update delay for next retry (exponential backoff)
                    retry_delay *= 2
                else:
                    logger.error(f"All {max_retries} attempts failed. Last error: {error_msg}")
                    issue = {
                        'type': 'connection_error',
                        'description': error_msg,
                    }
                    issue['severity'] = self._classify_issue_severity(issue)
                    self.validation_results['issues_found'].append(issue)
                    return {'status': 'error', 'message': str(e)}

        return {'status': 'error', 'message': f'Failed to connect after {max_retries} attempts'}

    def embed_query(self, query_text: str) -> List[float]:
        """
        Embed queries with Cohere for semantic search with timeout handling (T066).
        """
        try:
            response = self.cohere_client.embed(
                texts=[query_text],
                model=self.config['cohere_model'],
                input_type="search_query"
            )
            return response.embeddings[0]
        except requests.exceptions.Timeout:
            logger.error(f"Timeout embedding query '{query_text}': Request timed out after {self.config['cohere_timeout']} seconds")
            return []
        except requests.exceptions.ConnectionError:
            logger.error(f"Connection error embedding query '{query_text}': Unable to connect to Cohere API")
            return []
        except Exception as e:
            logger.error(f"Error embedding query '{query_text}': {str(e)}")
            return []

    def run_semantic_search(self, query_embedding: List[float], k: int = None, max_retries: int = 3, retry_delay: float = 1.0) -> List[Dict[str, Any]]:
        """
        Run semantic search (k=5–10) and return top-k results with retry logic for temporary unavailability.
        Also checks for schema changes during validation (T063) and handles network timeouts (T066).
        """
        # Use provided k or default from config
        k = k or self.config['test_query_k']

        for attempt in range(max_retries):
            try:
                # Check for schema changes before running search (T063)
                current_schema = self._capture_collection_schema()
                schema_issues = self._compare_collection_schema(current_schema)
                if schema_issues:
                    logger.warning(f"Schema changes detected during semantic search: {len(schema_issues)} issues")
                    self.validation_results['issues_found'].extend(schema_issues)

                search_results = self.qdrant_client.search(
                    collection_name=self.config['collection_name'],
                    query_vector=query_embedding,
                    limit=k,
                    with_payload=True,
                    with_vectors=False
                )

                # Format results
                formatted_results = []
                for result in search_results:
                    formatted_result = {
                        'id': result.id,
                        'score': result.score,
                        'payload': result.payload
                    }
                    formatted_results.append(formatted_result)

                return formatted_results

            except requests.exceptions.Timeout:
                error_msg = f"Timeout running semantic search: Request timed out after {self.config['qdrant_timeout']} seconds"
                logger.warning(f"Attempt {attempt + 1} failed: {error_msg}")

                if attempt < max_retries - 1:
                    # Wait before retrying
                    time.sleep(retry_delay)
                    # Update delay for next retry (exponential backoff)
                    retry_delay *= 2
                else:
                    logger.error(f"All {max_retries} attempts failed due to timeout. Last error: {error_msg}")
                    return []
            except requests.exceptions.ConnectionError:
                error_msg = "Connection error running semantic search: Unable to connect to Qdrant"
                logger.warning(f"Attempt {attempt + 1} failed: {error_msg}")

                if attempt < max_retries - 1:
                    # Wait before retrying
                    time.sleep(retry_delay)
                    # Update delay for next retry (exponential backoff)
                    retry_delay *= 2
                else:
                    logger.error(f"All {max_retries} attempts failed due to connection error. Last error: {error_msg}")
                    return []
            except Exception as e:
                error_msg = f"Error running semantic search: {str(e)}"
                logger.warning(f"Attempt {attempt + 1} failed: {error_msg}")

                if attempt < max_retries - 1:
                    # Wait before retrying
                    time.sleep(retry_delay)
                    # Update delay for next retry (exponential backoff)
                    retry_delay *= 2
                else:
                    logger.error(f"All {max_retries} attempts failed. Last error: {error_msg}")
                    return []

        return []

    def validate_metadata(self, chunk: Dict[str, Any]) -> Dict[str, bool]:
        """
        Inspect returned chunks for correctness, metadata accuracy, and relevance.
        """
        payload = chunk.get('payload', {})
        validation_results = {
            'url_present': bool(payload.get('url')),
            'title_present': bool(payload.get('title')),
            'content_preview_present': bool(payload.get('content_preview')),
            'original_content_length_present': 'original_content_length' in payload
        }

        # Overall metadata completeness
        validation_results['metadata_complete'] = all([
            validation_results['url_present'],
            validation_results['title_present'],
            validation_results['content_preview_present'],
            validation_results['original_content_length_present']
        ])

        return validation_results

    def check_for_issues(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, str]]:
        """
        Check for missing vectors, empty payloads, or schema mismatches.
        """
        issues = []

        # Track seen IDs for duplicate checking
        seen_ids = set()
        duplicate_ids = set()

        for chunk in chunks:
            chunk_id = chunk.get('id', 'unknown')
            payload = chunk.get('payload', {})

            # Check for duplicate IDs
            if chunk_id in seen_ids and chunk_id not in duplicate_ids:
                issue = {
                    'type': 'duplicate_id',
                    'chunk_id': chunk_id,
                    'description': f'Chunk ID appears multiple times: {chunk_id}'
                }
                issue['severity'] = self._classify_issue_severity(issue)
                issues.append(issue)
                duplicate_ids.add(chunk_id)
            else:
                seen_ids.add(chunk_id)

            # Check for empty or missing payloads
            if not payload:
                issue = {
                    'type': 'empty_payload',
                    'chunk_id': chunk_id,
                    'description': 'Chunk has empty or missing payload'
                }
                issue['severity'] = self._classify_issue_severity(issue)
                issues.append(issue)

            # Check for missing required fields
            required_fields = ['url', 'title', 'content_preview']
            for field in required_fields:
                if field not in payload or not payload[field]:
                    issue = {
                        'type': 'missing_field',
                        'chunk_id': chunk_id,
                        'field': field,
                        'description': f'Chunk missing required field: {field}'
                    }
                    issue['severity'] = self._classify_issue_severity(issue)
                    issues.append(issue)

        return issues

    def comprehensive_data_quality_check(self, max_retries: int = 3, retry_delay: float = 1.0) -> List[Dict[str, Any]]:
        """
        Perform comprehensive data quality checks including missing vectors,
        empty payloads, schema mismatches, broken vectors, and vector dimension validation
        with retry logic for temporary Qdrant unavailability.
        Also checks for schema changes during validation (T063) and handles network timeouts (T066).
        """
        issues = []

        # Get collection info to check vector count
        for attempt in range(max_retries):
            try:
                # Check for schema changes before running data quality checks (T063)
                current_schema = self._capture_collection_schema()
                schema_issues = self._compare_collection_schema(current_schema)
                if schema_issues:
                    logger.warning(f"Schema changes detected during data quality check: {len(schema_issues)} issues")
                    issues.extend(schema_issues)

                collection_info = self.qdrant_client.get_collection(self.config['collection_name'])
                vector_count = collection_info.points_count
                vector_config = collection_info.config.params.vectors

                # Check if vector count matches expected (T041)
                expected_count = self.config['expected_vector_count']  # Based on previous ingestion
                if vector_count != expected_count:
                    issues.append({
                        'type': 'vector_count_mismatch',
                        'expected': expected_count,
                        'actual': vector_count,
                        'description': f'Vector count mismatch: expected {expected_count}, got {vector_count}',
                        'severity': self._classify_issue_severity({
                            'type': 'vector_count_mismatch',
                            'expected': expected_count,
                            'actual': vector_count,
                            'description': f'Vector count mismatch: expected {expected_count}, got {vector_count}'
                        })
                    })

                # Check vector dimensions (T043)
                expected_dimension = 1024  # Cohere embedding size
                actual_dimension = vector_config.size if hasattr(vector_config, 'size') else 'unknown'
                if actual_dimension != expected_dimension:
                    issues.append({
                        'type': 'vector_dimension_mismatch',
                        'expected': expected_dimension,
                        'actual': actual_dimension,
                        'description': f'Vector dimension mismatch: expected {expected_dimension}, got {actual_dimension}',
                        'severity': self._classify_issue_severity({
                            'type': 'vector_dimension_mismatch',
                            'expected': expected_dimension,
                            'actual': actual_dimension,
                            'description': f'Vector dimension mismatch: expected {expected_dimension}, got {actual_dimension}'
                        })
                    })

                # Sample points to check for missing vectors, empty payloads, etc.
                # Use configured sample size to avoid performance issues
                sample_size = min(self.config['sample_size_for_data_quality'], vector_count)

                # Try to retrieve sample points with retry logic
                sample_ids = [str(i) for i in range(0, sample_size, max(1, sample_size//min(sample_size, self.config['max_query_samples'])))]

                # Retry the retrieve operation
                for retrieve_attempt in range(max_retries):
                    try:
                        sample_points = self.qdrant_client.retrieve(
                            collection_name=self.config['collection_name'],
                            ids=sample_ids,
                            with_payload=True,
                            with_vectors=True  # Include vectors to check for broken/invalid embeddings
                        )

                        for point in sample_points:
                            # Check for missing vectors (T037)
                            if not hasattr(point, 'vector') or point.vector is None:
                                issue = {
                                    'type': 'missing_vector',
                                    'chunk_id': point.id,
                                    'description': f'Chunk {point.id} has missing vector'
                                }
                                issue['severity'] = self._classify_issue_severity(issue)
                                issues.append(issue)

                            # Check for broken or invalid vectors (T040)
                            if hasattr(point, 'vector') and point.vector is not None:
                                vector = point.vector
                                # Check if vector is valid (not all zeros, not NaN, not infinite)
                                if isinstance(vector, list) and len(vector) > 0:
                                    import math
                                    # Check for NaN or infinite values
                                    has_invalid_values = any(
                                        val != val or math.isinf(val) for val in vector
                                    )
                                    if has_invalid_values:
                                        issue = {
                                            'type': 'broken_vector',
                                            'chunk_id': point.id,
                                            'description': f'Chunk {point.id} has invalid vector values (NaN or infinite)'
                                        }
                                        issue['severity'] = self._classify_issue_severity(issue)
                                        issues.append(issue)

                                    # Check if vector is all zeros (potentially broken)
                                    is_all_zeros = all(val == 0.0 for val in vector)
                                    if is_all_zeros:
                                        issue = {
                                            'type': 'broken_vector',
                                            'chunk_id': point.id,
                                            'description': f'Chunk {point.id} has all-zero vector (potentially broken)'
                                        }
                                        issue['severity'] = self._classify_issue_severity(issue)
                                        issues.append(issue)

                            # Check for empty payloads (T038)
                            if not point.payload:
                                issue = {
                                    'type': 'empty_payload',
                                    'chunk_id': point.id,
                                    'description': f'Chunk {point.id} has empty payload'
                                }
                                issue['severity'] = self._classify_issue_severity(issue)
                                issues.append(issue)

                            # Check for schema mismatches (T039)
                            if point.payload:
                                required_fields = ['url', 'title', 'content_preview']
                                for field in required_fields:
                                    if field not in point.payload or not point.payload[field]:
                                        issue = {
                                            'type': 'schema_mismatch',
                                            'chunk_id': point.id,
                                            'field': field,
                                            'description': f'Chunk {point.id} missing required field: {field}'
                                        }
                                        issue['severity'] = self._classify_issue_severity(issue)
                                        issues.append(issue)

                            # Check for duplicate IDs (T042) - this is already handled in check_for_issues
                            # but we'll add it here for completeness when checking the full dataset
                        break  # Break out of retrieve attempts if successful
                    except requests.exceptions.Timeout:
                        error_msg = f"Timeout retrieving samples: Request timed out after {self.config['qdrant_timeout']} seconds"
                        logger.warning(f"Retrieve attempt {retrieve_attempt + 1} failed: {error_msg}")
                        if retrieve_attempt < max_retries - 1:
                            time.sleep(retry_delay)
                            retry_delay *= 2
                        else:
                            issue = {
                                'type': 'sampling_timeout',
                                'description': error_msg
                            }
                            issue['severity'] = self._classify_issue_severity(issue)
                            issues.append(issue)
                    except requests.exceptions.ConnectionError:
                        error_msg = "Connection error retrieving samples: Unable to connect to Qdrant API"
                        logger.warning(f"Retrieve attempt {retrieve_attempt + 1} failed: {error_msg}")
                        if retrieve_attempt < max_retries - 1:
                            time.sleep(retry_delay)
                            retry_delay *= 2
                        else:
                            issue = {
                                'type': 'sampling_connection_error',
                                'description': error_msg
                            }
                            issue['severity'] = self._classify_issue_severity(issue)
                            issues.append(issue)
                    except Exception as retrieve_error:
                        logger.warning(f"Retrieve attempt {retrieve_attempt + 1} failed: {str(retrieve_error)}")
                        if retrieve_attempt < max_retries - 1:
                            time.sleep(retry_delay)
                            retry_delay *= 2
                        else:
                            issue = {
                                'type': 'sampling_error',
                                'description': f'Error during data quality sampling: {str(retrieve_error)}'
                            }
                            issue['severity'] = self._classify_issue_severity(issue)
                            issues.append(issue)
                break  # Break out of main attempts if successful
            except requests.exceptions.Timeout:
                error_msg = f"Timeout accessing collection for data quality checks: Request timed out after {self.config['qdrant_timeout']} seconds"
                logger.warning(f"Attempt {attempt + 1} failed: {error_msg}")

                if attempt < max_retries - 1:
                    # Wait before retrying
                    time.sleep(retry_delay)
                    # Update delay for next retry (exponential backoff)
                    retry_delay *= 2
                else:
                    logger.error(f"All {max_retries} attempts failed due to timeout. Last error: {error_msg}")
                    issue = {
                        'type': 'collection_access_timeout',
                        'description': error_msg
                    }
                    issue['severity'] = self._classify_issue_severity(issue)
                    issues.append(issue)
            except requests.exceptions.ConnectionError:
                error_msg = "Connection error accessing collection for data quality checks: Unable to connect to Qdrant API"
                logger.warning(f"Attempt {attempt + 1} failed: {error_msg}")

                if attempt < max_retries - 1:
                    # Wait before retrying
                    time.sleep(retry_delay)
                    # Update delay for next retry (exponential backoff)
                    retry_delay *= 2
                else:
                    logger.error(f"All {max_retries} attempts failed due to connection error. Last error: {error_msg}")
                    issue = {
                        'type': 'collection_access_connection_error',
                        'description': error_msg
                    }
                    issue['severity'] = self._classify_issue_severity(issue)
                    issues.append(issue)
            except Exception as e:
                error_msg = f"Error accessing collection for data quality checks: {str(e)}"
                logger.warning(f"Attempt {attempt + 1} failed: {error_msg}")

                if attempt < max_retries - 1:
                    # Wait before retrying
                    time.sleep(retry_delay)
                    # Update delay for next retry (exponential backoff)
                    retry_delay *= 2
                else:
                    logger.error(f"All {max_retries} attempts failed. Last error: {error_msg}")
                    issue = {
                        'type': 'collection_access_error',
                        'description': error_msg
                    }
                    issue['severity'] = self._classify_issue_severity(issue)
                    issues.append(issue)

        return issues

    def measure_latency(self, query: str, k: int = 5) -> Dict[str, float]:
        """
        Implement measure_latency function to measure total performance time (embedding + search + metadata hydration).
        """
        start_time = time.time()

        # Embed query time
        embed_start = time.time()
        query_embedding = self.embed_query(query)
        embed_time = time.time() - embed_start

        # Search time
        search_start = time.time()
        search_results = self.run_semantic_search(query_embedding, k)
        search_time = time.time() - search_start

        # Total time
        total_time = time.time() - start_time

        return {
            "total_time_ms": total_time * 1000,
            "embedding_time_ms": embed_time * 1000,
            "search_time_ms": search_time * 1000
        }

    def run_validation(self, max_retries: int = 3, retry_delay: float = 2.0):
        """
        Execute the full validation pipeline with graceful degradation and retry logic for failed validation attempts (T065).
        Enhanced with comprehensive logging throughout the validation pipeline (T070).
        """
        logger.info("Starting RAG retrieval validation pipeline...")

        for attempt in range(max_retries):
            try:
                logger.info(f"Validation attempt {attempt + 1}/{max_retries}")

                # Reset validation results for each attempt
                self.validation_results = {
                    'validation_timestamp': datetime.now().isoformat(),
                    'collection_health': {},
                    'retrieval_metrics': [],
                    'metadata_integrity': {},
                    'performance_metrics': {},
                    'quality_metrics': {},
                    'issues_found': []
                }

                # Log configuration values at start of validation
                logger.info(f"Using configuration: collection_name={self.config['collection_name']}, "
                           f"test_query_k={self.config['test_query_k']}, "
                           f"latency_target={self.config['latency_target_ms']}ms, "
                           f"total_test_queries={len(self.test_queries)}")

                # Phase 1: Verify collection health
                logger.info("Phase 1: Verifying collection health")
                try:
                    collection_health = self.connect_and_verify_collection()
                    self.validation_results['collection_health'] = collection_health

                    if collection_health.get('status') == 'error':
                        logger.error("Collection verification failed, but continuing with degraded functionality")
                        # Continue with degraded functionality instead of stopping
                        collection_health = {
                            'collection_exists': False,
                            'vector_count': 0,
                            'vector_dimensions': 'unknown',
                            'distance_type': 'unknown',
                            'status': 'degraded'
                        }
                        self.validation_results['collection_health'] = collection_health
                    else:
                        logger.info(f"Collection health verified: {collection_health.get('vector_count', 0)} vectors, "
                                   f"{collection_health.get('vector_dimensions', 'unknown')} dimensions")
                except Exception as e:
                    logger.error(f"Collection verification failed with error: {str(e)}, continuing with degraded functionality")
                    collection_health = {
                        'collection_exists': False,
                        'vector_count': 0,
                        'vector_dimensions': 'unknown',
                        'distance_type': 'unknown',
                        'status': 'degraded'
                    }
                    self.validation_results['collection_health'] = collection_health

                # Phase 2: Run semantic search tests
                logger.info("Phase 2: Running semantic search tests")
                logger.info(f"Processing {len(self.test_queries)} test queries")

                # Check for schema changes before starting query tests (T063)
                current_schema = self._capture_collection_schema()
                schema_issues = self._compare_collection_schema(current_schema)
                if schema_issues:
                    logger.warning(f"Schema changes detected before query tests: {len(schema_issues)} issues")
                    self.validation_results['issues_found'].extend(schema_issues)

                total_retrieval_time = 0
                total_queries = len(self.test_queries)
                metadata_validations = []
                all_issues = []
                successful_queries = 0

                for i, query in enumerate(self.test_queries):
                    try:
                        logger.info(f"Processing query {i+1}/{total_queries}: {query['text'][:50]}...")

                        # Validate the query first (T064)
                        is_valid, validation_error = self._validate_query(query['text'])
                        if not is_valid:
                            logger.warning(f"Invalid query detected: {validation_error}")
                            all_issues.append({
                                'type': 'malformed_query',
                                'query_id': query['id'],
                                'query_text': query['text'],
                                'description': validation_error,
                                'severity': self._classify_issue_severity({
                                    'type': 'malformed_query',
                                    'query_id': query['id'],
                                    'query_text': query['text'],
                                    'description': validation_error
                                })
                            })
                            continue  # Skip this query

                        # Embed the query
                        query_embedding = self.embed_query(query['text'])
                        if not query_embedding:
                            logger.warning(f"Failed to embed query: {query['text']}, skipping...")
                            continue
                        else:
                            logger.debug(f"Successfully embedded query {query['id']}, embedding length: {len(query_embedding)}")

                        # Measure retrieval latency (pass the query text instead of embedding)
                        latency_metrics = self.measure_latency(query['text'], k=self.config['test_query_k'])
                        retrieval_time = latency_metrics['total_time_ms']
                        total_retrieval_time += retrieval_time

                        # Run semantic search
                        search_results = self.run_semantic_search(query_embedding, k=self.config['test_query_k'])

                        # Log search results statistics
                        logger.debug(f"Query {query['id']} returned {len(search_results)} results, "
                                   f"avg retrieval time: {retrieval_time:.2f}ms")

                        # Check if query returned no results (T062)
                        if not search_results:
                            logger.warning(f"Query '{query['text']}' returned no results")
                            all_issues.append({
                                'type': 'no_results_for_query',
                                'query_id': query['id'],
                                'query_text': query['text'],
                                'description': f"Query '{query['text']}' returned no results",
                                'severity': self._classify_issue_severity({
                                    'type': 'no_results_for_query',
                                    'query_id': query['id'],
                                    'query_text': query['text'],
                                    'description': f"Query '{query['text']}' returned no results"
                                })
                            })

                        # Validate metadata for each result
                        for result in search_results:
                            try:
                                metadata_validation = self.validate_metadata(result)
                                metadata_validations.append(metadata_validation)

                                # Check for issues
                                issues = self.check_for_issues([result])
                                all_issues.extend(issues)
                            except Exception as e:
                                logger.error(f"Error validating metadata for result {result.get('id', 'unknown')}: {str(e)}")
                                # Add issue for metadata validation failure
                                all_issues.append({
                                    'type': 'metadata_validation_error',
                                    'description': f'Error validating metadata: {str(e)}',
                                    'severity': self._classify_issue_severity({
                                        'type': 'metadata_validation_error',
                                        'description': f'Error validating metadata: {str(e)}'
                                    })
                                })

                        # Calculate relevance accuracy for this query
                        relevance_accuracy = self.calculate_relevance_accuracy(query, search_results)

                        # Store retrieval metrics
                        retrieval_metric = {
                            'query_id': query['id'],
                            'query_text': query['text'],
                            'query_type': query['type'],
                            'results_count': len(search_results),
                            'avg_retrieval_time_ms': retrieval_time,
                            'top_result_score': search_results[0]['score'] if search_results else 0,
                            'relevance_accuracy': relevance_accuracy
                        }
                        self.validation_results['retrieval_metrics'].append(retrieval_metric)
                        successful_queries += 1

                        # Log progress periodically
                        if (i + 1) % 5 == 0:
                            logger.info(f"Progress: {i+1}/{total_queries} queries processed")
                    except Exception as e:
                        logger.error(f"Error processing query {query['id']}: {str(e)}, continuing...")
                        # Add issue for query processing failure but continue
                        all_issues.append({
                            'type': 'query_processing_error',
                            'query_id': query['id'],
                            'description': f'Error processing query: {str(e)}',
                            'severity': self._classify_issue_severity({
                                'type': 'query_processing_error',
                                'description': f'Error processing query: {str(e)}'
                            })
                        })

                # Phase 3: Calculate metrics
                logger.info("Phase 3: Calculating validation metrics")
                logger.info(f"Calculated metrics for {successful_queries}/{total_queries} successful queries")

                # Performance metrics - calculate only if we have successful queries
                avg_retrieval_time = total_retrieval_time / successful_queries if successful_queries > 0 else 0
                self.validation_results['performance_metrics'] = {
                    'total_queries_executed': successful_queries,
                    'total_queries_attempted': total_queries,
                    'successful_query_percentage': (successful_queries / total_queries * 100) if total_queries > 0 else 0,
                    'avg_retrieval_time_ms': avg_retrieval_time,
                    'max_retrieval_time_ms': max([m['avg_retrieval_time_ms'] for m in self.validation_results['retrieval_metrics']], default=0) if self.validation_results['retrieval_metrics'] else 0,
                    'min_retrieval_time_ms': min([m['avg_retrieval_time_ms'] for m in self.validation_results['retrieval_metrics']], default=0) if self.validation_results['retrieval_metrics'] else 0,
                    'latency_target_met': avg_retrieval_time < self.config['latency_target_ms'] if avg_retrieval_time > 0 else False
                }

                # Log performance metrics
                logger.info(f"Performance metrics - Avg retrieval time: {avg_retrieval_time:.2f}ms, "
                           f"Max: {self.validation_results['performance_metrics']['max_retrieval_time_ms']:.2f}ms, "
                           f"Latency target met: {self.validation_results['performance_metrics']['latency_target_met']}")

                # Metadata integrity metrics
                total_metadata_checks = len(metadata_validations)
                complete_metadata_count = sum(1 for m in metadata_validations if m.get('metadata_complete', False))
                metadata_integrity_percentage = (complete_metadata_count / total_metadata_checks * 100) if total_metadata_checks > 0 else 0

                self.validation_results['metadata_integrity'] = {
                    'total_checks': total_metadata_checks,
                    'complete_metadata_count': complete_metadata_count,
                    'integrity_percentage': metadata_integrity_percentage,
                    'integrity_target_met': metadata_integrity_percentage >= self.config['metadata_integrity_target'],
                    'field_completeness': {
                        'url_present': sum(1 for m in metadata_validations if m.get('url_present', False)) / total_metadata_checks if total_metadata_checks > 0 else 0,
                        'title_present': sum(1 for m in metadata_validations if m.get('title_present', False)) / total_metadata_checks if total_metadata_checks > 0 else 0,
                        'content_preview_present': sum(1 for m in metadata_validations if m.get('content_preview_present', False)) / total_metadata_checks if total_metadata_checks > 0 else 0,
                        'original_content_length_present': sum(1 for m in metadata_validations if m.get('original_content_length_present', False)) / total_metadata_checks if total_metadata_checks > 0 else 0
                    }
                }

                # Log metadata integrity metrics
                logger.info(f"Metadata integrity - {metadata_integrity_percentage:.2f}% complete, "
                           f"Target met: {self.validation_results['metadata_integrity']['integrity_target_met']}")

                # Quality metrics
                # Calculate average relevance accuracy across all queries
                relevance_accuracies = [m['relevance_accuracy'] for m in self.validation_results['retrieval_metrics'] if 'relevance_accuracy' in m]
                avg_relevance_accuracy = sum(relevance_accuracies) / len(relevance_accuracies) if relevance_accuracies else 0

                self.validation_results['quality_metrics'] = {
                    'total_chunks_validated': total_metadata_checks,
                    'total_queries_processed': successful_queries,
                    'issues_found_count': len(all_issues),
                    'issues_by_type': {},
                    'collection_vector_count': collection_health.get('vector_count', 0),
                    'expected_vector_count': self.config['expected_vector_count'],  # Based on previous ingestion
                    'vector_count_match': collection_health.get('vector_count', 0) == self.config['expected_vector_count'],
                    'avg_relevance_accuracy': avg_relevance_accuracy,
                    'relevance_target_met': avg_relevance_accuracy >= self.config['relevance_accuracy_target']  # Target configurable relevance accuracy
                }

                # Log quality metrics
                logger.info(f"Quality metrics - Avg relevance accuracy: {avg_relevance_accuracy:.2f}%, "
                           f"Target met: {self.validation_results['quality_metrics']['relevance_target_met']}")

                # Count issues by type
                for issue in all_issues:
                    issue_type = issue['type']
                    if issue_type not in self.validation_results['quality_metrics']['issues_by_type']:
                        self.validation_results['quality_metrics']['issues_by_type'][issue_type] = 0
                    self.validation_results['quality_metrics']['issues_by_type'][issue_type] += 1

                # Add all issues to main results
                self.validation_results['issues_found'].extend(all_issues)

                # Log issue summary
                logger.info(f"Found {len(all_issues)} issues during validation: "
                           f"{self.validation_results['quality_metrics']['issues_found_count']} total issues")

                # Phase 4: Run comprehensive data quality checks (T037-T044)
                logger.info("Phase 4: Running comprehensive data quality checks")
                try:
                    data_quality_issues = self.comprehensive_data_quality_check()
                    logger.info(f"Data quality checks found {len(data_quality_issues)} issues")
                    self.validation_results['issues_found'].extend(data_quality_issues)
                except Exception as e:
                    logger.error(f"Error during comprehensive data quality checks: {str(e)}, continuing...")
                    # Add issue for data quality check failure but continue
                    self.validation_results['issues_found'].append({
                        'type': 'data_quality_check_error',
                        'description': f'Error during comprehensive data quality checks: {str(e)}',
                        'severity': self._classify_issue_severity({
                            'type': 'data_quality_check_error',
                            'description': f'Error during comprehensive data quality checks: {str(e)}'
                        })
                    })

                # Document data quality findings (T044)
                self._document_data_quality_findings(self.validation_results['issues_found'])

                # Phase 5: Validate behavior under high query load conditions (T067)
                logger.info("Phase 5: Validating behavior under high query load conditions")
                try:
                    logger.info(f"Starting high load validation with {10} concurrent queries and {5} query batch size")
                    load_validation_result = self.validate_high_load_conditions(
                        num_concurrent_queries=10,  # Default: 10 concurrent queries
                        query_batch_size=5         # Default: 5 queries per batch
                    )

                    # Add load validation metrics to main results
                    self.validation_results['load_validation'] = load_validation_result

                    # Add any issues found during load testing to main issues list
                    if load_validation_result.get('issues_found'):
                        logger.info(f"High load validation found {len(load_validation_result['issues_found'])} issues")
                        self.validation_results['issues_found'].extend(load_validation_result['issues_found'])
                    else:
                        logger.info("High load validation completed with no issues detected")

                    logger.info(f"Load validation completed with {load_validation_result['success_rate_percentage']:.2f}% success rate, "
                               f"avg response time: {load_validation_result['load_metrics']['avg_response_time_ms']:.2f}ms, "
                               f"throughput: {load_validation_result['load_metrics']['throughput_queries_per_second']:.2f} queries/sec")
                except Exception as e:
                    logger.error(f"Error during high load validation: {str(e)}, continuing...")
                    # Add issue for load validation failure but continue
                    self.validation_results['issues_found'].append({
                        'type': 'load_validation_error',
                        'description': f'Error during high load validation: {str(e)}',
                        'severity': self._classify_issue_severity({
                            'type': 'load_validation_error',
                            'description': f'Error during high load validation: {str(e)}'
                        })
                    })

                # Final summary logging
                total_issues = len(self.validation_results['issues_found'])
                successful_queries = self.validation_results['quality_metrics']['total_queries_processed']
                total_queries = len(self.test_queries)

                logger.info(f"Validation completed with graceful degradation where needed!")
                logger.info(f"Summary: {successful_queries}/{total_queries} queries processed successfully, "
                           f"{total_issues} total issues found")

                # Log top issue types
                if self.validation_results['issues_found']:
                    issue_types = {}
                    for issue in self.validation_results['issues_found']:
                        issue_type = issue.get('type', 'unknown')
                        issue_types[issue_type] = issue_types.get(issue_type, 0) + 1

                    top_issues = sorted(issue_types.items(), key=lambda x: x[1], reverse=True)[:5]
                    logger.info(f"Top issue types: {top_issues}")

                return self.validation_results

            except Exception as e:
                logger.error(f"Validation attempt {attempt + 1} failed with error: {str(e)}")

                if attempt < max_retries - 1:
                    logger.info(f"Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                    retry_delay *= 2  # Exponential backoff
                else:
                    logger.error(f"All {max_retries} validation attempts failed.")
                    # Even if all retries failed, return the last attempt's results
                    if not hasattr(self, 'validation_results') or not self.validation_results:
                        self.validation_results = {
                            'validation_timestamp': datetime.now().isoformat(),
                            'collection_health': {},
                            'retrieval_metrics': [],
                            'metadata_integrity': {},
                            'performance_metrics': {},
                            'quality_metrics': {},
                            'issues_found': [{
                                'type': 'validation_failure',
                                'description': f'All {max_retries} validation attempts failed: {str(e)}',
                                'severity': 'high'
                            }]
                        }
                    return self.validation_results

    def generate_report(self, output_file: str = None) -> Dict[str, Any]:
        """
        Generate a retrieval quality report summarizing findings.
        """
        # Generate default filename with timestamp if not provided
        if output_file is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_file = f"validation_report_{timestamp}.json"

        logger.info(f"Generating validation report to {output_file}")

        # Create diagnostic report structure based on data model (T045)
        diagnostic_report = {
            # DiagnosticReport fields from data model
            'validation_timestamp': self.validation_results['validation_timestamp'],
            'total_queries_run': len(self.validation_results['retrieval_metrics']),
            'avg_retrieval_latency': self.validation_results['performance_metrics'].get('avg_retrieval_time_ms', 0),
            'metadata_integrity_percentage': self.validation_results['metadata_integrity'].get('integrity_percentage', 0),
            'relevance_accuracy': self.validation_results['quality_metrics'].get('avg_relevance_accuracy', 0),
            'collection_health_status': self.validation_results['collection_health'].get('status', 'unknown'),
            'missing_vectors_count': self._count_issues_by_type('missing_vector'),
            'performance_metrics': self._format_performance_metrics(),
            'quality_metrics': self._format_quality_metrics(),
            'issues_found': self.validation_results['issues_found'],
            'issues_summary': self._generate_issues_summary(),  # Add issues summary with severity classification (T051)

            # Additional summary for clarity
            'summary': {
                'status': 'completed',
                'collection_health_ok': self.validation_results['collection_health'].get('status') != 'error',
                'latency_target_met': self.validation_results['performance_metrics'].get('latency_target_met', False),
                'metadata_integrity_target_met': self.validation_results['metadata_integrity'].get('integrity_target_met', False),
                'issues_found': len(self.validation_results['issues_found']) > 0,
                'total_issues': len(self.validation_results['issues_found'])
            }
        }

        # Write to file
        with open(output_file, 'w') as f:
            json.dump(diagnostic_report, f, indent=2)

        # Also save raw validation results
        raw_results_file = output_file.replace('.json', '_raw.json')
        with open(raw_results_file, 'w') as f:
            json.dump(self.validation_results, f, indent=2)

        # Print summary to console
        self._print_summary({'summary': diagnostic_report['summary'], 'detailed_results': diagnostic_report})

        # Persist the validation results for potential comparison
        self._persist_validation_results(output_file, diagnostic_report)

        return diagnostic_report

    def _persist_validation_results(self, output_file: str, diagnostic_report: Dict[str, Any]):
        """
        Persist validation results for potential comparison with future runs (T060).
        """
        try:
            # Create a results history directory if it doesn't exist
            history_dir = "validation_history"
            if not os.path.exists(history_dir):
                os.makedirs(history_dir)

            # Save the current results in the history directory
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            history_file = os.path.join(history_dir, f"validation_{timestamp}.json")

            with open(history_file, 'w') as f:
                json.dump(diagnostic_report, f, indent=2)

            # Keep only the last 10 validation results to avoid cluttering
            self._cleanup_old_validation_results(history_dir)

            logger.info(f"Validation results persisted to {history_file}")
        except Exception as e:
            logger.error(f"Error persisting validation results: {str(e)}")

    def _cleanup_old_validation_results(self, history_dir: str):
        """
        Keep only the last 10 validation results to avoid cluttering.
        """
        try:
            validation_files = [f for f in os.listdir(history_dir) if f.startswith("validation_") and f.endswith(".json")]
            validation_files.sort()

            # Remove oldest files if we have more than 10
            if len(validation_files) > 10:
                files_to_remove = validation_files[:-10]
                for file in files_to_remove:
                    os.remove(os.path.join(history_dir, file))
                    logger.debug(f"Removed old validation result file: {file}")
        except Exception as e:
            logger.error(f"Error cleaning up old validation results: {str(e)}")

    def compare_with_previous_run(self, current_report: Dict[str, Any], previous_report_path: str = None) -> Dict[str, Any]:
        """
        Compare current validation results with a previous run (T060).
        """
        if previous_report_path is None:
            # Look for the most recent validation report in history
            history_dir = "validation_history"
            if not os.path.exists(history_dir):
                logger.info("No previous validation results found for comparison")
                return {}

            validation_files = [f for f in os.listdir(history_dir) if f.startswith("validation_") and f.endswith(".json")]
            if not validation_files:
                logger.info("No previous validation results found for comparison")
                return {}

            # Get the most recent validation file
            validation_files.sort(reverse=True)
            previous_report_path = os.path.join(history_dir, validation_files[0])

        try:
            with open(previous_report_path, 'r') as f:
                previous_report = json.load(f)

            comparison = {
                'comparison_timestamp': datetime.now().isoformat(),
                'current_run_timestamp': current_report.get('validation_timestamp'),
                'previous_run_timestamp': previous_report.get('validation_timestamp'),
                'metrics_comparison': {
                    'avg_retrieval_latency_change': current_report.get('avg_retrieval_latency', 0) - previous_report.get('avg_retrieval_latency', 0),
                    'metadata_integrity_percentage_change': current_report.get('metadata_integrity_percentage', 0) - previous_report.get('metadata_integrity_percentage', 0),
                    'relevance_accuracy_change': current_report.get('relevance_accuracy', 0) - previous_report.get('relevance_accuracy', 0),
                    'total_issues_change': len(current_report.get('issues_found', [])) - len(previous_report.get('issues_found', []))
                },
                'regression_detected': self._detect_regressions(current_report, previous_report)
            }

            logger.info(f"Comparison completed with previous run from {previous_report_path}")
            return comparison

        except Exception as e:
            logger.error(f"Error comparing with previous run: {str(e)}")
            return {}

    def _detect_regressions(self, current_report: Dict[str, Any], previous_report: Dict[str, Any]) -> Dict[str, bool]:
        """
        Detect potential regressions by comparing metrics.
        """
        regression_thresholds = {
            'latency_increase': 10.0,  # Allow 10ms increase before flagging
            'integrity_decrease': 1.0,  # Allow 1% decrease before flagging
            'accuracy_decrease': 1.0,   # Allow 1% decrease before flagging
            'issue_increase': 1         # Allow 1 more issue before flagging
        }

        current_latency = current_report.get('avg_retrieval_latency', 0)
        previous_latency = previous_report.get('avg_retrieval_latency', 0)

        current_integrity = current_report.get('metadata_integrity_percentage', 0)
        previous_integrity = previous_report.get('metadata_integrity_percentage', 0)

        current_accuracy = current_report.get('relevance_accuracy', 0)
        previous_accuracy = previous_report.get('relevance_accuracy', 0)

        current_issues = len(current_report.get('issues_found', []))
        previous_issues = len(previous_report.get('issues_found', []))

        regressions = {
            'latency_regression': (current_latency - previous_latency) > regression_thresholds['latency_increase'],
            'integrity_regression': (previous_integrity - current_integrity) > regression_thresholds['integrity_decrease'],
            'accuracy_regression': (previous_accuracy - current_accuracy) > regression_thresholds['accuracy_decrease'],
            'issue_count_regression': (current_issues - previous_issues) > regression_thresholds['issue_increase']
        }

        return regressions

    def _count_issues_by_type(self, issue_type: str) -> int:
        """
        Count issues of a specific type in the validation results.
        """
        count = 0
        for issue in self.validation_results['issues_found']:
            if issue.get('type') == issue_type:
                count += 1
        return count

    def _format_performance_metrics(self) -> Dict[str, Any]:
        """
        Format performance metrics according to the data model.
        """
        return {
            'total_queries_executed': self.validation_results['performance_metrics'].get('total_queries_executed', 0),
            'avg_retrieval_time_ms': self.validation_results['performance_metrics'].get('avg_retrieval_time_ms', 0),
            'max_retrieval_time_ms': self.validation_results['performance_metrics'].get('max_retrieval_time_ms', 0),
            'min_retrieval_time_ms': self.validation_results['performance_metrics'].get('min_retrieval_time_ms', 0),
            'latency_target_met': self.validation_results['performance_metrics'].get('latency_target_met', False)
        }

    def _format_quality_metrics(self) -> Dict[str, Any]:
        """
        Format quality metrics according to the data model.
        """
        quality_metrics = {
            'total_chunks_validated': self.validation_results['quality_metrics'].get('total_chunks_validated', 0),
            'issues_found_count': self.validation_results['quality_metrics'].get('issues_found_count', 0),
            'issues_by_type': self.validation_results['quality_metrics'].get('issues_by_type', {}),
            'collection_vector_count': self.validation_results['quality_metrics'].get('collection_vector_count', 0),
            'expected_vector_count': self.validation_results['quality_metrics'].get('expected_vector_count', 0),
            'vector_count_match': self.validation_results['quality_metrics'].get('vector_count_match', False),
            'avg_relevance_accuracy': self.validation_results['quality_metrics'].get('avg_relevance_accuracy', 0),
            'relevance_target_met': self.validation_results['quality_metrics'].get('relevance_target_met', False)
        }

        # Include load validation metrics if available
        if 'load_validation' in self.validation_results:
            load_metrics = self.validation_results['load_validation'].get('load_metrics', {})
            quality_metrics['load_validation'] = {
                'concurrent_requests': load_metrics.get('concurrent_requests', 0),
                'success_rate_percentage': load_metrics.get('success_rate_percentage', 0),
                'avg_response_time_ms': load_metrics.get('avg_response_time_ms', 0),
                'min_response_time_ms': load_metrics.get('min_response_time_ms', 0),
                'max_response_time_ms': load_metrics.get('max_response_time_ms', 0),
                'throughput_queries_per_second': load_metrics.get('throughput_queries_per_second', 0)
            }

        return quality_metrics

    def _generate_issues_summary(self) -> Dict[str, Any]:
        """
        Generate issues summary with severity classification (T051).
        """
        issues = self.validation_results['issues_found']

        # Count issues by severity
        severity_counts = {'high': 0, 'medium': 0, 'low': 0, 'unknown': 0}
        severity_details = {'high': [], 'medium': [], 'low': [], 'unknown': []}

        for issue in issues:
            severity = issue.get('severity', 'unknown')
            if severity in severity_counts:
                severity_counts[severity] += 1
                severity_details[severity].append(issue)
            else:
                severity_counts['unknown'] += 1
                severity_details['unknown'].append(issue)

        return {
            'total_issues': len(issues),
            'by_severity': severity_counts,
            'high_severity_issues': severity_details['high'],
            'medium_severity_issues': severity_details['medium'],
            'low_severity_issues': severity_details['low'],
            'unknown_severity_issues': severity_details['unknown'],
            'has_high_severity_issues': severity_counts['high'] > 0,
            'has_critical_issues': severity_counts['high'] > 0  # treating high as critical
        }

    def _print_summary(self, report: Dict[str, Any]):
        """
        Print a human-readable summary of the validation results.
        """
        print("\n" + "="*60)
        print("RAG RETRIEVAL VALIDATION REPORT")
        print("="*60)

        summary = report['summary']
        results = report['detailed_results']

        print(f"Validation Timestamp: {results['validation_timestamp']}")
        print(f"Total Queries Tested: {results.get('total_queries_run', 0)}")
        print(f"Collection Health: {'✅ OK' if summary['collection_health_ok'] else '❌ ERROR'}")
        print(f"Latency Target Met (<200ms): {'✅ YES' if summary['latency_target_met'] else '❌ NO'}")
        print(f"Metadata Integrity Target Met (≥99%): {'✅ YES' if summary['metadata_integrity_target_met'] else '❌ NO'}")
        print(f"Issues Found: {'❌ YES' if summary['issues_found'] else '✅ NO'} ({summary['total_issues']} total)")

        if results['performance_metrics']:
            perf = results['performance_metrics']
            print(f"\nPerformance Metrics:")
            print(f"  Avg Retrieval Time: {perf['avg_retrieval_time_ms']:.2f} ms")
            print(f"  Max Retrieval Time: {perf['max_retrieval_time_ms']:.2f} ms")
            print(f"  Min Retrieval Time: {perf['min_retrieval_time_ms']:.2f} ms")

        if results['metadata_integrity']:
            meta = results['metadata_integrity']
            print(f"\nMetadata Integrity:")
            print(f"  Integrity Percentage: {meta['integrity_percentage']:.2f}%")
            print(f"  Total Chunks Checked: {meta['total_checks']}")
            print(f"  Complete Metadata: {meta['complete_metadata_count']}")

        if results['quality_metrics']:
            quality = results['quality_metrics']
            print(f"\nQuality Metrics:")
            print(f"  Vector Count Match: {'✅ YES' if quality['vector_count_match'] else '❌ NO'}")
            print(f"  Expected: 35, Actual: {quality['collection_vector_count']}")
            print(f"  Avg Relevance Accuracy: {quality['avg_relevance_accuracy']:.2f}%")
            print(f"  Relevance Target Met (>90%): {'✅ YES' if quality['relevance_target_met'] else '❌ NO'}")

        if results['issues_found']:
            print(f"\nIssues Found:")
            for issue in results['issues_found'][:10]:  # Show first 10 issues
                print(f"  - {issue['type']}: {issue['description']}")
            if len(results['issues_found']) > 10:
                print(f"  ... and {len(results['issues_found']) - 10} more issues")

        print("="*60)

    def calculate_relevance_accuracy(self, query: Dict[str, Any], results: List[Dict[str, Any]]) -> float:
        """
        Implement relevance accuracy calculation comparing results against expected topics for test queries.
        """
        # Extract expected topic from the query
        expected_topic = query.get('expected_topic', '').lower()

        # Calculate relevance based on how well results match the expected topic
        relevant_results = 0
        total_results = len(results)

        for result in results:
            # Check if result content or metadata contains the expected topic
            content = result.get('payload', {}).get('content', '').lower()
            title = result.get('payload', {}).get('title', '').lower()

            if expected_topic in content or expected_topic in title:
                relevant_results += 1

        # Return relevance accuracy as a percentage
        return (relevant_results / total_results * 100) if total_results > 0 else 0

    def _document_data_quality_findings(self, issues: List[Dict[str, Any]]) -> None:
        """
        Document data quality findings with specific examples (T044).
        """
        if not issues:
            logger.info("No data quality issues found to document")
            return

        logger.info(f"Documenting {len(issues)} data quality findings with specific examples")

        # Create a summary of issues by type
        issues_by_type = {}
        for issue in issues:
            issue_type = issue.get('type', 'unknown')
            if issue_type not in issues_by_type:
                issues_by_type[issue_type] = []
            issues_by_type[issue_type].append(issue)

        # Log examples of each issue type
        for issue_type, issue_list in issues_by_type.items():
            logger.info(f"Data quality issue type: {issue_type} ({len(issue_list)} occurrences)")

            # Show first few examples as specific examples
            for i, issue in enumerate(issue_list[:3]):  # Show first 3 examples
                logger.info(f"  Example {i+1}: {issue.get('description', 'No description')}")

    def _capture_collection_schema(self) -> Dict[str, Any]:
        """
        Capture the current collection schema for comparison during validation (T063) with timeout handling (T066).
        """
        try:
            collection_info = self.qdrant_client.get_collection(self.config['collection_name'])

            # Extract schema information
            schema = {
                'vector_size': collection_info.config.params.vectors.size if hasattr(collection_info.config.params.vectors, 'size') else 'unknown',
                'distance_type': collection_info.config.params.vectors.distance if hasattr(collection_info.config.params.vectors, 'distance') else 'unknown',
                'payload_schema': collection_info.config.params.vectors if hasattr(collection_info.config.params, 'vectors') else {},
                'collection_config': {
                    'hnsw_config': collection_info.config.hnsw_config.dict() if hasattr(collection_info.config, 'hnsw_config') and collection_info.config.hnsw_config else None,
                    'optimizer_config': collection_info.config.optimizer_config.dict() if hasattr(collection_info.config, 'optimizer_config') and collection_info.config.optimizer_config else None,
                    'wal_config': collection_info.config.wal_config.dict() if hasattr(collection_info.config, 'wal_config') and collection_info.config.wal_config else None
                }
            }

            return schema
        except requests.exceptions.Timeout:
            logger.error(f"Timeout capturing collection schema: Request timed out after {self.config['qdrant_timeout']} seconds")
            return {}
        except requests.exceptions.ConnectionError:
            logger.error("Connection error capturing collection schema: Unable to connect to Qdrant API")
            return {}
        except Exception as e:
            logger.error(f"Error capturing collection schema: {str(e)}")
            return {}

    def _compare_collection_schema(self, current_schema: Dict[str, Any]) -> List[Dict[str, str]]:
        """
        Compare current schema with initial schema and detect changes (T063).
        """
        issues = []

        if not self.initial_collection_schema:
            # If no initial schema, capture it now and return no issues
            self.initial_collection_schema = current_schema
            return issues

        # Compare vector size
        initial_size = self.initial_collection_schema.get('vector_size')
        current_size = current_schema.get('vector_size')
        if initial_size != current_size:
            issues.append({
                'type': 'vector_size_change',
                'description': f'Vector size changed from {initial_size} to {current_size}',
                'severity': self._classify_issue_severity({
                    'type': 'vector_size_change',
                    'description': f'Vector size changed from {initial_size} to {current_size}'
                })
            })

        # Compare distance type
        initial_distance = self.initial_collection_schema.get('distance_type')
        current_distance = current_schema.get('distance_type')
        if initial_distance != current_distance:
            issues.append({
                'type': 'distance_type_change',
                'description': f'Distance type changed from {initial_distance} to {current_distance}',
                'severity': self._classify_issue_severity({
                    'type': 'distance_type_change',
                    'description': f'Distance type changed from {initial_distance} to {current_distance}'
                })
            })

        # Compare other schema elements as needed
        # For now, we're focusing on the most critical schema elements

        return issues

    def _validate_query(self, query_text: str) -> Tuple[bool, str]:
        """
        Validate query for length and potential malformation (T064) with input sanitization (T069).
        Returns (is_valid, error_message).
        """
        # Sanitize the input query text
        query_text = self._sanitize_input(query_text)

        # Check for extremely long queries (e.g., more than 1000 characters)
        if len(query_text) > 1000:
            return False, f"Query exceeds maximum length of 1000 characters (actual: {len(query_text)} characters)"

        # Check for empty query
        if not query_text or query_text.strip() == "":
            return False, "Query is empty or contains only whitespace"

        # Check for potentially malformed queries with excessive special characters
        special_chars_ratio = sum(1 for c in query_text if not c.isalnum() and not c.isspace()) / len(query_text)
        if special_chars_ratio > 0.5:  # More than 50% special characters
            return False, f"Query contains excessive special characters ({special_chars_ratio:.2%} are special characters)"

        # Check for potentially problematic patterns
        if query_text.count('..') > 2:  # Multiple consecutive dots
            return False, "Query contains multiple consecutive dots which may be malformed"

        if query_text.count('  ') > 2:  # Multiple consecutive spaces
            return False, "Query contains multiple consecutive spaces which may be malformed"

        # Check for potential injection patterns
        potential_injection_patterns = ['<script', 'javascript:', 'SELECT ', 'DROP ', 'INSERT ', 'DELETE ']
        for pattern in potential_injection_patterns:
            if pattern.lower() in query_text.lower():
                return False, f"Query contains potential injection pattern: {pattern}"

        # Query is valid
        return True, ""

    def _sanitize_input(self, input_text: str) -> str:
        """
        Sanitize input text by removing potentially harmful characters/sequences (T069).
        """
        if not input_text:
            return input_text

        # Remove null bytes
        sanitized = input_text.replace('\x00', '')

        # Remove control characters except common whitespace
        sanitized = ''.join(c for c in sanitized if ord(c) >= 32 or c in '\t\n\r')

        # Limit consecutive whitespace to single space
        import re
        sanitized = re.sub(r'\s+', ' ', sanitized)

        # Strip leading/trailing whitespace
        sanitized = sanitized.strip()

        return sanitized

    def validate_high_load_conditions(self, num_concurrent_queries: int = 10, query_batch_size: int = 5) -> Dict[str, Any]:
        """
        Validate behavior under high query load conditions (T067).
        Tests the system's performance when handling multiple concurrent queries.
        Enhanced with input validation and sanitization (T072).
        """
        import concurrent.futures
        from threading import Lock

        # Validate input parameters first (T072)
        try:
            num_concurrent_queries = int(num_concurrent_queries)
            query_batch_size = int(query_batch_size)
        except (ValueError, TypeError):
            logger.error(f"Invalid parameters for validate_high_load_conditions: "
                        f"num_concurrent_queries={num_concurrent_queries}, query_batch_size={query_batch_size}")
            return {
                'test_type': 'high_load_validation',
                'error': 'Invalid parameter types',
                'issues_found': [{
                    'type': 'invalid_parameter_type',
                    'description': f"num_concurrent_queries and query_batch_size must be integers",
                    'severity': self._classify_issue_severity({
                        'type': 'invalid_parameter_type',
                        'description': f"num_concurrent_queries and query_batch_size must be integers"
                    })
                }]
            }

        # Additional parameter validation
        if num_concurrent_queries <= 0 or num_concurrent_queries > 100:
            logger.error(f"num_concurrent_queries must be between 1 and 100, got {num_concurrent_queries}")
            return {
                'test_type': 'high_load_validation',
                'error': 'Invalid num_concurrent_queries value',
                'issues_found': [{
                    'type': 'invalid_parameter_value',
                    'description': f"num_concurrent_queries must be between 1 and 100, got {num_concurrent_queries}",
                    'severity': self._classify_issue_severity({
                        'type': 'invalid_parameter_value',
                        'description': f"num_concurrent_queries must be between 1 and 100, got {num_concurrent_queries}"
                    })
                }]
            }

        if query_batch_size <= 0 or query_batch_size > 50:
            logger.error(f"query_batch_size must be between 1 and 50, got {query_batch_size}")
            return {
                'test_type': 'high_load_validation',
                'error': 'Invalid query_batch_size value',
                'issues_found': [{
                    'type': 'invalid_parameter_value',
                    'description': f"query_batch_size must be between 1 and 50, got {query_batch_size}",
                    'severity': self._classify_issue_severity({
                        'type': 'invalid_parameter_value',
                        'description': f"query_batch_size must be between 1 and 50, got {query_batch_size}"
                    })
                }]
            }

        logger.info(f"Starting high load validation with {num_concurrent_queries} concurrent queries")

        # Use a subset of test queries for load testing to avoid excessive load
        test_queries_subset = self.test_queries[:query_batch_size] if len(self.test_queries) >= query_batch_size else self.test_queries

        # Shared results container with thread safety
        results_lock = Lock()
        all_results = []
        all_issues = []
        load_metrics = {
            'total_queries_sent': 0,
            'successful_queries': 0,
            'failed_queries': 0,
            'avg_response_time_ms': 0,
            'min_response_time_ms': float('inf'),
            'max_response_time_ms': 0,
            'throughput_queries_per_second': 0,
            'concurrent_requests': num_concurrent_queries,
            'start_time': time.time()
        }

        def run_single_query(query):
            """Run a single query and return results with timing."""
            start_time = time.time()
            query_result = {
                'query_id': query['id'],
                'query_text': query['text'],
                'start_time': start_time,
                'success': False,
                'response_time_ms': 0,
                'error': None
            }

            try:
                # Validate the query first
                is_valid, validation_error = self._validate_query(query['text'])
                if not is_valid:
                    query_result['error'] = validation_error
                    query_result['success'] = False
                else:
                    # Embed the query
                    query_embedding = self.embed_query(query['text'])
                    if not query_embedding:
                        query_result['error'] = "Failed to embed query"
                        query_result['success'] = False
                    else:
                        # Run semantic search
                        search_results = self.run_semantic_search(query_embedding, k=self.config['test_query_k'])

                        query_result['success'] = True
                        query_result['search_results_count'] = len(search_results)
                        query_result['top_score'] = search_results[0]['score'] if search_results else 0

            except Exception as e:
                query_result['error'] = str(e)
                query_result['success'] = False

            query_result['response_time_ms'] = (time.time() - start_time) * 1000

            # Update metrics
            with results_lock:
                load_metrics['total_queries_sent'] += 1
                if query_result['success']:
                    load_metrics['successful_queries'] += 1
                else:
                    load_metrics['failed_queries'] += 1
                    # Add issue for failed query
                    all_issues.append({
                        'type': 'high_load_query_failure',
                        'query_id': query['id'],
                        'query_text': query['text'],
                        'error': query_result['error'],
                        'response_time_ms': query_result['response_time_ms'],
                        'severity': self._classify_issue_severity({
                            'type': 'high_load_query_failure',
                            'query_id': query['id'],
                            'query_text': query['text'],
                            'error': query_result['error']
                        })
                    })

                # Update timing metrics
                if query_result['response_time_ms'] < load_metrics['min_response_time_ms']:
                    load_metrics['min_response_time_ms'] = query_result['response_time_ms']
                if query_result['response_time_ms'] > load_metrics['max_response_time_ms']:
                    load_metrics['max_response_time_ms'] = query_result['response_time_ms']

                all_results.append(query_result)

            return query_result

        # Execute queries concurrently
        start_time = time.time()
        with concurrent.futures.ThreadPoolExecutor(max_workers=num_concurrent_queries) as executor:
            # Submit all query tasks
            future_to_query = {executor.submit(run_single_query, query): query for query in test_queries_subset}

            # Wait for all queries to complete
            for future in concurrent.futures.as_completed(future_to_query):
                pass  # Results are already collected in the run_single_query function

        total_time = time.time() - start_time
        total_successful = load_metrics['successful_queries']

        # Calculate final metrics
        if total_successful > 0:
            successful_results = [r for r in all_results if r['success']]
            if successful_results:
                load_metrics['avg_response_time_ms'] = sum(r['response_time_ms'] for r in successful_results) / len(successful_results)
            load_metrics['throughput_queries_per_second'] = total_successful / total_time if total_time > 0 else 0
        else:
            load_metrics['avg_response_time_ms'] = 0
            load_metrics['throughput_queries_per_second'] = 0

        # If no successful queries, set min to 0
        if load_metrics['min_response_time_ms'] == float('inf'):
            load_metrics['min_response_time_ms'] = 0

        # Add load validation results to main validation results
        load_validation_result = {
            'test_type': 'high_load_validation',
            'concurrent_requests': num_concurrent_queries,
            'query_batch_size': query_batch_size,
            'total_time_seconds': total_time,
            'load_metrics': load_metrics,
            'issues_found': all_issues,
            'success_rate_percentage': (load_metrics['successful_queries'] / load_metrics['total_queries_sent'] * 100) if load_metrics['total_queries_sent'] > 0 else 0
        }

        logger.info(f"High load validation completed. Success rate: {load_validation_result['success_rate_percentage']:.2f}%, "
                   f"Avg response time: {load_metrics['avg_response_time_ms']:.2f}ms, "
                   f"Throughput: {load_metrics['throughput_queries_per_second']:.2f} queries/sec")

        return load_validation_result

    def _classify_issue_severity(self, issue: Dict[str, Any]) -> str:
        """
        Classify issue severity based on issue type and impact.
        """
        issue_type = issue.get('type', 'unknown')

        # Define severity classification rules
        high_severity_types = [
            'collection_missing', 'connection_error', 'vector_count_mismatch',
            'missing_vector', 'broken_vector', 'collection_access_error',
            'vector_size_change', 'distance_type_change', 'schema_change',
            'connection_timeout', 'collection_access_timeout', 'sampling_timeout'
        ]

        medium_severity_types = [
            'sampling_error', 'empty_payload', 'schema_mismatch',
            'sampling_connection_error', 'collection_access_connection_error'
        ]

        low_severity_types = [
            'duplicate_id', 'missing_field'
        ]

        if issue_type in high_severity_types:
            return 'high'
        elif issue_type in medium_severity_types:
            return 'medium'
        elif issue_type in low_severity_types:
            return 'low'
        else:
            # Default to medium for unknown issue types
            return 'medium'

    def cleanup_resources(self):
        """
        Clean up resources such as closing connections and releasing resources (T073).
        """
        logger.info("Cleaning up resources...")

        # Close Qdrant client connection if it exists
        if hasattr(self, 'qdrant_client') and self.qdrant_client:
            try:
                # QdrantClient has a close method that closes the underlying HTTP client
                if hasattr(self.qdrant_client, 'close'):
                    self.qdrant_client.close()
                logger.info("Qdrant client connection closed")
            except Exception as e:
                logger.error(f"Error closing Qdrant client: {str(e)}")

        # For Cohere client, there's typically no explicit close method
        # The client doesn't maintain persistent connections in the same way
        # We'll just log that we're done with it
        if hasattr(self, 'cohere_client'):
            logger.info("Cohere client reference cleared")

        # Close the requests session
        if 'session' in globals():
            try:
                session.close()
                logger.info("Global requests session closed")
            except Exception as e:
                logger.error(f"Error closing requests session: {str(e)}")


def main():
    """
    Main function to execute the RAG retrieval validation.
    """
    logger.info("Initializing RAG Retrieval Validator")

    # Create validator with configuration
    validator = RetrievalValidator()

    try:
        # Validate environment variables (T071)
        logger.info("Validating environment variables...")
        env_issues = validator.validate_environment_variables()
        if env_issues:
            logger.warning(f"Environment validation found {len(env_issues)} issues:")
            for issue in env_issues:
                logger.warning(f"  - {issue['type']}: {issue['description']}")

            # Add environment validation issues to the validation results
            validator.validation_results['issues_found'].extend(env_issues)

        # Check for critical missing environment variables before proceeding
        critical_missing = []
        for var in ['COHERE_API_KEY', 'QDRANT_API_KEY', 'QDRANT_URL']:
            if not os.getenv(var):
                critical_missing.append(var)

        if critical_missing:
            logger.error(f"Missing required environment variables: {critical_missing}")
            return

        # Run validation
        validation_results = validator.run_validation(max_retries=3, retry_delay=2.0)

        # Generate report
        report = validator.generate_report("validation_report.json")

        # Check if we should attempt to fix minor data issues
        issues = validation_results['issues_found']
        if issues:
            logger.info(f"Found {len(issues)} issues. Checking for minor fixable issues...")

            # For now, just report issues - we'll implement fixes as needed
            for issue in issues:
                logger.info(f"Issue: {issue['type']} - {issue['description']}")

        return report

    finally:
        # Ensure resources are cleaned up
        logger.info("Performing resource cleanup...")
        validator.cleanup_resources()


if __name__ == "__main__":
    main()