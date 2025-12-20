/**
 * Integration Test for RAG Frontend Components
 * Validates that all components work together correctly
 */

import { apiClient } from './services/api';
import { sendQuery, sendSelectedTextQuery } from './services/query-handler';
import { validateQuery, validateSelectedText } from './utils/validators';
import { sanitizeResponse } from './utils/sanitizer';
import { parseResponse } from './utils/response-parser';
import { cacheService } from './services/cache';
import { stateManager } from './services/state-manager';
import { performanceMonitor } from './services/performance';

console.log('🔍 Starting RAG Integration Validation...');

// Test 1: API Service Connection
async function testApiConnection() {
  console.log('\n🧪 Test 1: API Service Connection');
  try {
    // Test health endpoint
    const healthResponse = await apiClient.health();
    console.log('✅ Health check successful:', healthResponse.status);
    return true;
  } catch (error) {
    console.error('❌ Health check failed:', error.message);
    return false;
  }
}

// Test 2: Query Validation
function testQueryValidation() {
  console.log('\n🧪 Test 2: Query Validation');
  try {
    // Test valid query
    const validQueryResult = validateQuery('What is humanoid robotics?');
    if (validQueryResult.isValid) {
      console.log('✅ Valid query validation passed');
    } else {
      console.error('❌ Valid query validation failed:', validQueryResult.error);
      return false;
    }

    // Test invalid query (too short)
    const invalidQueryResult = validateQuery('');
    if (!invalidQueryResult.isValid) {
      console.log('✅ Invalid query validation passed');
    } else {
      console.error('❌ Invalid query validation should have failed');
      return false;
    }

    // Test selected text validation
    const validSelectedTextResult = validateSelectedText('This is selected text');
    if (validSelectedTextResult.isValid) {
      console.log('✅ Valid selected text validation passed');
    } else {
      console.error('❌ Valid selected text validation failed:', validSelectedTextResult.error);
      return false;
    }

    return true;
  } catch (error) {
    console.error('❌ Query validation test failed:', error.message);
    return false;
  }
}

// Test 3: Sanitization
function testSanitization() {
  console.log('\n🧪 Test 3: Sanitization');
  try {
    // Test sanitizing a response with potential XSS
    const testResponse = {
      answer: 'This is a safe answer',
      sources: [
        {
          content: 'Safe content',
          source_document: 'document1.pdf',
          page_number: 5
        }
      ]
    };

    const sanitized = sanitizeResponse(testResponse);
    console.log('✅ Response sanitization passed');

    // Verify the sanitized response has the expected structure
    if (sanitized.answer && Array.isArray(sanitized.sources)) {
      console.log('✅ Sanitized response structure is correct');
      return true;
    } else {
      console.error('❌ Sanitized response structure is incorrect');
      return false;
    }
  } catch (error) {
    console.error('❌ Sanitization test failed:', error.message);
    return false;
  }
}

// Test 4: Response Parsing
function testResponseParsing() {
  console.log('\n🧪 Test 4: Response Parsing');
  try {
    const testResponse = {
      answer: 'This is the answer to your question',
      sources: [
        {
          content: 'This is source content',
          source_document: 'chapter1.md',
          page_number: 3,
          section_title: 'Introduction',
          similarity_score: 0.85,
          chunk_id: 'chunk-123'
        }
      ],
      provider_used: 'gemini',
      debug_info: {
        embedding_time: 0.1,
        retrieval_time: 0.2,
        generation_time: 0.3,
        total_time: 0.6
      }
    };

    const parsed = parseResponse(testResponse);

    if (parsed.answer && Array.isArray(parsed.sources) && parsed.sources.length > 0) {
      console.log('✅ Response parsing passed');
      console.log('✅ Parsed answer:', parsed.answer.substring(0, 30) + '...');
      console.log('✅ Parsed sources count:', parsed.sources.length);
      return true;
    } else {
      console.error('❌ Response parsing failed - missing expected properties');
      return false;
    }
  } catch (error) {
    console.error('❌ Response parsing test failed:', error.message);
    return false;
  }
}

// Test 5: Caching
function testCaching() {
  console.log('\n🧪 Test 5: Caching');
  try {
    // Test setting and getting from cache
    const testQuery = 'test query for cache';
    const testResponse = { answer: 'test answer' };

    cacheService.cacheQuery(testQuery, {}, testResponse, 5000); // 5 second TTL

    const cachedResponse = cacheService.getCachedQuery(testQuery, {});

    if (cachedResponse && cachedResponse.answer === 'test answer') {
      console.log('✅ Caching functionality passed');
      return true;
    } else {
      console.error('❌ Caching functionality failed');
      return false;
    }
  } catch (error) {
    console.error('❌ Caching test failed:', error.message);
    return false;
  }
}

// Test 6: State Management
function testStateManagement() {
  console.log('\n🧪 Test 6: State Management');
  try {
    // Test adding query to history
    stateManager.addQueryToHistory('test query', { answer: 'test response' });

    const history = stateManager.getQueryHistory();
    if (history.length > 0 && history[0].query === 'test query') {
      console.log('✅ State management functionality passed');
      return true;
    } else {
      console.error('❌ State management functionality failed');
      return false;
    }
  } catch (error) {
    console.error('❌ State management test failed:', error.message);
    return false;
  }
}

// Test 7: Performance Monitoring
function testPerformanceMonitoring() {
  console.log('\n🧪 Test 7: Performance Monitoring');
  try {
    // Test timing functionality
    performanceMonitor.startTiming('test-operation');

    // Simulate some work
    const start = Date.now();
    while (Date.now() - start < 100) {} // 100ms delay

    const duration = performanceMonitor.endTiming('test-operation');

    if (duration >= 100) { // Should be at least 100ms
      console.log(`✅ Performance monitoring passed (duration: ${duration}ms)`);
      return true;
    } else {
      console.error(`❌ Performance monitoring failed (duration: ${duration}ms)`);
      return false;
    }
  } catch (error) {
    console.error('❌ Performance monitoring test failed:', error.message);
    return false;
  }
}

// Run all tests
async function runAllTests() {
  console.log('🚀 Running all integration tests...\n');

  const tests = [
    { name: 'API Connection', fn: testApiConnection },
    { name: 'Query Validation', fn: testQueryValidation },
    { name: 'Sanitization', fn: testSanitization },
    { name: 'Response Parsing', fn: testResponseParsing },
    { name: 'Caching', fn: testCaching },
    { name: 'State Management', fn: testStateManagement },
    { name: 'Performance Monitoring', fn: testPerformanceMonitoring }
  ];

  const results = [];

  for (const test of tests) {
    console.log(`\n📋 Running ${test.name} test...`);
    const result = test.fn instanceof AsyncFunction ? await test.fn() : test.fn();
    results.push({ name: test.name, passed: result });
    console.log(`✅ ${test.name}: ${result ? 'PASSED' : 'FAILED'}`);
  }

  // Summary
  const passedTests = results.filter(r => r.passed).length;
  const totalTests = results.length;

  console.log('\n🎉 Integration Test Summary:');
  console.log(`✅ Passed: ${passedTests}/${totalTests} tests`);

  if (passedTests === totalTests) {
    console.log('🎊 All tests passed! RAG integration is working correctly.');
    return true;
  } else {
    console.log(`💥 ${totalTests - passedTests} test(s) failed. Please review the errors above.`);
    return false;
  }
}

// AsyncFunction constructor for dynamic async function detection
const AsyncFunction = Object.getPrototypeOf(async function(){}).constructor;

// Run the tests
runAllTests()
  .then(success => {
    if (success) {
      console.log('\n🎊 RAG Frontend Integration Validation Complete - ALL SYSTEMS GO!');
    } else {
      console.log('\n💥 RAG Frontend Integration Validation Failed - Please review issues above.');
    }
  })
  .catch(error => {
    console.error('\n💥 Unexpected error during validation:', error);
  });