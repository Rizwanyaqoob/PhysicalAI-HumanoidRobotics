"""
Test script for RAG Retrieval Pipeline Validation

This script demonstrates how to run the complete validation pipeline with the existing RAG_embeddings collection.
It includes example usage, expected outputs, and validation of requirements from spec.md.
"""

import os
import sys
import json
from datetime import datetime

# Add the backend directory to the path to import the module
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from retrieval_validator import RetrievalValidator

def test_validation_pipeline():
    """
    Test the complete validation pipeline with the existing RAG_embeddings collection.
    This function demonstrates the proper usage and checks that requirements are satisfied.
    """
    print("Starting RAG Retrieval Pipeline Validation Test")
    print("="*60)

    # Check for required environment variables
    required_vars = ['COHERE_API_KEY', 'QDRANT_API_KEY', 'QDRANT_URL']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"ERROR: Missing required environment variables: {missing_vars}")
        print("Please set these variables before running the validation:")
        print("- COHERE_API_KEY: Your Cohere API key")
        print("- QDRANT_API_KEY: Your Qdrant API key")
        print("- QDRANT_URL: Your Qdrant Cloud URL")
        return False

    print("All required environment variables are present")

    try:
        # Create validator instance
        print("\n1. Creating RetrievalValidator instance...")
        validator = RetrievalValidator()
        print("Validator created successfully")

        # Validate environment variables
        print("\n2. Validating environment variables...")
        env_issues = validator.validate_environment_variables()
        if env_issues:
            print(f"! Found {len(env_issues)} environment validation issues:")
            for issue in env_issues:
                print(f"   - {issue['type']}: {issue['description']}")
        else:
            print("Environment variables validation passed")

        # Run the validation pipeline
        print("\n3. Running validation pipeline...")
        validation_results = validator.run_validation(max_retries=3, retry_delay=2.0)
        print("Validation pipeline completed")

        # Generate report
        print("\n4. Generating validation report...")
        report = validator.generate_report("test_validation_report.json")
        print("Validation report generated: test_validation_report.json")

        # Display summary
        print("\n5. Validation Summary:")
        print(f"   - Validation Timestamp: {report['validation_timestamp']}")
        print(f"   - Total Queries Run: {report['total_queries_run']}")
        print(f"   - Avg Retrieval Latency: {report['avg_retrieval_latency']:.2f} ms")
        print(f"   - Metadata Integrity: {report['metadata_integrity_percentage']:.2f}%")
        print(f"   - Relevance Accuracy: {report['relevance_accuracy']:.2f}%")
        print(f"   - Collection Health: {report['collection_health_status']}")
        print(f"   - Missing Vectors Count: {report['missing_vectors_count']}")
        print(f"   - Total Issues Found: {report['summary']['total_issues']}")

        # Validate requirements from spec.md
        print("\n6. Validating Requirements from spec.md:")
        requirements_satisfied = validate_requirements(validation_results, report)

        # Clean up resources
        print("\n7. Cleaning up resources...")
        validator.cleanup_resources()
        print("Resources cleaned up successfully")

        print("\n" + "="*60)
        print("VALIDATION TEST COMPLETED")
        if requirements_satisfied:
            print("All requirements satisfied")
            return True
        else:
            print("Some requirements not satisfied")
            return False

    except Exception as e:
        print(f"ERROR: Validation failed with exception: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def validate_requirements(validation_results, report):
    """
    Validate that all requirements from spec.md are satisfied by validation results.
    """
    print("   Validating Functional Requirements:")

    all_satisfied = True

    # FR-001: System MUST connect to Qdrant Cloud and verify the existence of the RAG_embeddings collection
    collection_exists = validation_results['collection_health'].get('status') != 'error'
    print(f"   - FR-001: Qdrant collection exists: {'PASS' if collection_exists else 'FAIL'}")
    if not collection_exists:
        all_satisfied = False

    # FR-002: System MUST verify the schema of the Qdrant collection matches expected vector dimensions and metadata structure
    # This is handled during validation, check if there were schema-related issues
    schema_issues = [issue for issue in validation_results['issues_found'] if 'schema' in issue.get('type', '')]
    schema_ok = len(schema_issues) == 0
    print(f"   - FR-002: Schema validation passed: {'PASS' if schema_ok else 'FAIL'}")
    if not schema_ok:
        all_satisfied = False
        print(f"     Schema issues found: {len(schema_issues)}")

    # FR-003: System MUST count and verify the total number of vectors in the collection matches expected ingestion count
    vector_count_match = validation_results['quality_metrics'].get('vector_count_match', False)
    print(f"   - FR-003: Vector count matches expected: {'PASS' if vector_count_match else 'FAIL'}")
    if not vector_count_match:
        all_satisfied = False

    # FR-004: System MUST execute semantic search tests with multiple query types (broad and specific)
    queries_executed = validation_results['performance_metrics'].get('total_queries_executed', 0) > 0
    print(f"   - FR-004: Semantic search tests executed: {'PASS' if queries_executed else 'FAIL'}")
    if not queries_executed:
        all_satisfied = False

    # FR-005: System MUST validate metadata integrity for each retrieved chunk (URL, title, content preview)
    metadata_integrity_percentage = validation_results['metadata_integrity'].get('integrity_percentage', 0)
    print(f"   - FR-005: Metadata integrity validated: {'PASS' if metadata_integrity_percentage > 0 else 'FAIL'}")

    # FR-006: System MUST benchmark retrieval latency and ensure total time stays below 200ms per query
    avg_latency = validation_results['performance_metrics'].get('avg_retrieval_time_ms', float('inf'))
    latency_target_met = validation_results['performance_metrics'].get('latency_target_met', False)
    print(f"   - FR-006: Latency < 200ms: {'PASS' if latency_target_met else 'FAIL'} (actual: {avg_latency:.2f}ms)")
    if not latency_target_met:
        all_satisfied = False

    # FR-007: System MUST identify and report any missing, corrupted, or empty vectors in the collection
    missing_vectors_count = report['missing_vectors_count']
    print(f"   - FR-007: Missing vectors reported: {'PASS' if missing_vectors_count >= 0 else 'FAIL'} (count: {missing_vectors_count})")

    # FR-009: System MUST validate that 99% or more of chunks have complete and correct metadata
    metadata_target_met = validation_results['metadata_integrity'].get('integrity_target_met', False)
    print(f"   - FR-009: Metadata integrity >= 99%: {'PASS' if metadata_target_met else 'FAIL'} (actual: {metadata_integrity_percentage:.2f}%)")
    if not metadata_target_met:
        all_satisfied = False

    # FR-010: System MUST ensure zero missing chunks or empty vectors exist in the collection
    missing_vectors_ok = missing_vectors_count == 0
    print(f"   - FR-010: Zero missing vectors: {'PASS' if missing_vectors_ok else 'FAIL'} (count: {missing_vectors_count})")
    if not missing_vectors_ok:
        all_satisfied = False

    print("\n   Validating Success Criteria:")

    # SC-001: Semantic search consistently returns correct -k chunks with relevance accuracy > 90%
    relevance_accuracy = validation_results['quality_metrics'].get('avg_relevance_accuracy', 0)
    relevance_target_met = validation_results['quality_metrics'].get('relevance_target_met', False)
    print(f"   - SC-001: Relevance accuracy > 90%: {'PASS' if relevance_target_met else 'FAIL'} (actual: {relevance_accuracy:.2f}%)")
    if not relevance_target_met:
        all_satisfied = False

    # SC-002: Metadata integrity is ≥ 99% (fields present and correct)
    print(f"   - SC-002: Metadata integrity >= 99%: {'PASS' if metadata_target_met else 'FAIL'} (actual: {metadata_integrity_percentage:.2f}%)")
    if not metadata_target_met:
        all_satisfied = False

    # SC-003: Retrieval latency is < 200 ms per query on Qdrant Free Tier
    print(f"   - SC-003: Latency < 200ms: {'PASS' if latency_target_met else 'FAIL'} (actual: {avg_latency:.2f}ms)")
    if not latency_target_met:
        all_satisfied = False

    # SC-004: Zero missing chunks or empty vectors exist in the collection
    print(f"   - SC-004: Zero missing vectors: {'PASS' if missing_vectors_ok else 'FAIL'} (count: {missing_vectors_count})")
    if not missing_vectors_ok:
        all_satisfied = False

    return all_satisfied

def run_sample_validation():
    """
    Run a sample validation with example configuration to demonstrate usage.
    """
    print("Running Sample Validation (without actual API calls)...")
    print("This demonstrates the validation flow and expected structure.")

    # Create a sample validation report structure
    sample_report = {
        'validation_timestamp': datetime.now().isoformat(),
        'total_queries_run': 10,
        'avg_retrieval_latency': 150.0,  # ms
        'metadata_integrity_percentage': 99.5,
        'relevance_accuracy': 95.0,
        'collection_health_status': 'healthy',
        'missing_vectors_count': 0,
        'performance_metrics': {
            'total_queries_executed': 10,
            'avg_retrieval_time_ms': 150.0,
            'max_retrieval_time_ms': 180.0,
            'min_retrieval_time_ms': 120.0,
            'latency_target_met': True
        },
        'quality_metrics': {
            'total_chunks_validated': 50,
            'issues_found_count': 2,
            'issues_by_type': {'low_severity': 2},
            'collection_vector_count': 35,
            'expected_vector_count': 35,
            'vector_count_match': True,
            'avg_relevance_accuracy': 95.0,
            'relevance_target_met': True
        },
        'issues_found': [
            {
                'type': 'low_severity_issue',
                'description': 'Minor issue found during validation',
                'severity': 'low'
            },
            {
                'type': 'low_severity_issue',
                'description': 'Another minor issue found',
                'severity': 'low'
            }
        ],
        'issues_summary': {
            'total_issues': 2,
            'by_severity': {'high': 0, 'medium': 0, 'low': 2, 'unknown': 0},
            'has_high_severity_issues': False,
            'has_critical_issues': False
        },
        'summary': {
            'status': 'completed',
            'collection_health_ok': True,
            'latency_target_met': True,
            'metadata_integrity_target_met': True,
            'issues_found': True,
            'total_issues': 2
        }
    }

    print("Sample validation report structure created successfully")
    print(f"Sample report includes {sample_report['total_queries_run']} queries")
    print(f"Average latency: {sample_report['avg_retrieval_latency']} ms")
    print(f"Metadata integrity: {sample_report['metadata_integrity_percentage']}%")
    print(f"Relevance accuracy: {sample_report['relevance_accuracy']}%")

    # Write sample report to file
    with open('sample_validation_report.json', 'w') as f:
        json.dump(sample_report, f, indent=2)

    print("Sample validation report saved as 'sample_validation_report.json'")

    return sample_report

if __name__ == "__main__":
    print("RAG Retrieval Pipeline Validation - Test Suite")
    print("="*60)

    # First, run a sample validation to show expected structure
    print("\nStep 1: Running sample validation (demonstration only)")
    sample_report = run_sample_validation()

    # Then, if environment is properly configured, run actual validation
    print("\nStep 2: Testing actual validation pipeline")
    success = test_validation_pipeline()

    print("\n" + "="*60)
    if success:
        print("VALIDATION TESTS COMPLETED SUCCESSFULLY")
        print("The RAG retrieval pipeline validation is working correctly.")
    else:
        print("VALIDATION TESTS HAD ISSUES")
        print("Check the output above for details on what needs to be addressed.")

    print("="*60)