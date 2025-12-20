# Implementation Plan: Frontend RAG Integration

**Feature**: Frontend RAG Integration
**Branch**: `3-frontend-rag-integration`
**Created**: 2025-12-14
**Status**: Draft

## Overview

This plan outlines the implementation of the frontend integration with the backend RAG agent service. The goal is to connect the published Docusaurus book frontend with the FastAPI RAG backend to enable real-time question answering based on book content. The implementation will follow the "connect → send → render → validate" approach.

## Architecture

### Technical Context
- Frontend: Docusaurus-based book UI (JavaScript/TypeScript)
- Backend: FastAPI RAG agent service (Python)
- Communication: HTTP/HTTPS API calls
- API Contract: `/api/ask` endpoint for question submission
- Response Format: JSON with answer and source references

### System Components
1. **Frontend Service**: JavaScript module to handle API communication
2. **Query Handler**: Function to send user queries to backend
3. **Response Renderer**: Component to display answers and sources
4. **State Manager**: Handle loading, error, and success states
5. **UI Component**: Sidebar interface for question input and response display

## Implementation Strategy

### Phase 1: Backend API Connection
- Configure environment variable for backend API endpoint
- Implement HTTP client for API communication
- Establish CORS-compatible communication
- Test connectivity with backend health check

### Phase 2: Query Submission
- Implement query handler for full-book questions
- Implement query handler for selected-text questions
- Add request validation and sanitization
- Implement request queuing for multiple queries

### Phase 3: Response Handling
- Parse backend JSON responses
- Render answers with proper formatting
- Display source document references and page numbers
- Implement response caching for repeated queries

### Phase 4: User Experience
- Add loading indicators during query processing
- Implement error handling with user-friendly messages
- Add empty state handling for no results
- Ensure responses render without page refresh

## Technical Specifications

### API Contract
- **Ask Endpoint**: `POST /api/ask`
  - Request Body:
    ```json
    {
      "query": "string (1-10000 chars)",
      "provider": "gemini" (pattern: ^gemini$),
      "max_chunks": "number (default: 5, range: 1-20)",
      "user_id": "string (optional)"
    }
    ```
  - Response:
    ```json
    {
      "answer": "string",
      "sources": [
        {
          "content": "string",
          "source_document": "string",
          "page_number": "number (optional)",
          "section_title": "string (optional)",
          "similarity_score": "number (0.0-1.0)",
          "chunk_id": "string"
        }
      ],
      "provider_used": "gemini",
      "debug_info": {
        "embedding_time": "number",
        "retrieval_time": "number",
        "generation_time": "number",
        "total_time": "number"
      }
    }
    ```

- **Retrieve Endpoint**: `POST /api/retrieve`
  - Request Body:
    ```json
    {
      "query": "string (1-10000 chars)",
      "max_chunks": "number (default: 5, range: 1-20)"
    }
    ```
  - Response:
    ```json
    {
      "chunks": [
        {
          "content": "string",
          "source_document": "string",
          "page_number": "number (optional)",
          "section_title": "string (optional)",
          "similarity_score": "number (0.0-1.0)",
          "chunk_id": "string"
        }
      ],
      "retrieval_time": "number"
    }
    ```

- **Health Endpoint**: `GET /health`
  - Response:
    ```json
    {
      "status": "healthy|unhealthy",
      "timestamp": "string",
      "dependencies": {
        "string": "connected|disconnected"
      }
    }
    ```

- **Error Responses**:
  - 400 Bad Request: Invalid input parameters
  - 500 Internal Server Error: Processing errors

### Frontend Components
1. **QueryInput Component**: Text area for user questions
2. **SubmitButton Component**: Trigger for sending queries
3. **ResponseDisplay Component**: Container for answers and sources
4. **LoadingIndicator Component**: Visual feedback during processing
5. **ErrorDisplay Component**: Error messages for various failure scenarios

### Error Handling
- Network errors: "Unable to connect to the question-answering service. Please check your connection and try again."
- Backend failures: "The question-answering service is temporarily unavailable. Please try again in a moment."
- Empty responses: "No relevant content found in the book for your question. Try rephrasing or asking about a different topic."

## Implementation Steps

### Step 1: Environment Setup
1. Add `BACKEND_API_URL` environment variable for backend endpoint (default: http://localhost:8009)
2. Configure Docusaurus environment variables in `docusaurus.config.js` or `.env` file
3. Implement environment-based configuration switching for development/production
4. Set up proper CORS configuration in backend to allow frontend domain

### Step 2: HTTP Client Implementation
1. Create API service module with fetch/axios client
2. Implement request/response interceptors for authentication and error handling
3. Add timeout handling (default 30 seconds)
4. Implement retry logic for failed requests (max 3 retries)
5. Add request cancellation for long-running queries
6. Implement proper error handling for different HTTP status codes

### Step 3: Query Handler Development
1. Create `sendQuery` function for full-book questions
   - Format request body with query, provider ("gemini"), max_chunks (default: 5)
   - Send POST request to `/api/ask` endpoint
   - Handle response parsing and error cases
2. Create `sendSelectedTextQuery` function for context-restricted queries
   - Accept selected text as additional context parameter
   - Prepend selected text to query as context: "Based on the following text: [selected text], [original question]"
   - Send appropriate request to backend using same `/api/ask` endpoint
   - Include context_type parameter to indicate selected-text mode
3. Add input validation and sanitization (query length: 1-10000 chars)
4. Implement request queuing for multiple simultaneous queries (max 10 per session)
5. Add request rate limiting to prevent excessive backend calls
6. Implement proper content sanitization to prevent XSS attacks

### Step 4: Response Processing
1. Create response parser for backend JSON
   - Parse answer text from `response.answer` field
   - Extract source references from `response.sources` array
   - Process timing information from `response.debug_info` for performance metrics
2. Implement source reference formatting
   - Display source document with page number if available
   - Format section titles for better context
   - Show similarity scores to indicate relevance
   - Create clickable links to source documents where possible
3. Add content sanitization for XSS protection
   - Sanitize HTML content in answers and source content
   - Implement proper escaping for special characters
   - Validate URLs in source references
4. Create caching mechanism for repeated queries
   - Implement in-memory cache with query-answer pairs
   - Set cache expiration (e.g., 10 minutes)
   - Limit cache size to prevent memory issues
5. Implement response formatting
   - Format answer text with proper line breaks and paragraphs
   - Style source references with appropriate CSS
   - Create expandable sections for detailed source information

### Step 5: UI Component Development
1. Design sidebar component for question input
   - Create text area for user questions (with character counter)
   - Add submit button with appropriate labeling
   - Include option for selected text context (checkbox or separate input)
   - Add query history panel to show recent questions
2. Implement response display with proper formatting
   - Create dedicated area for answer text with scrollable container if needed
   - Design source reference panel with expandable sections
   - Implement proper text formatting (paragraphs, lists, code blocks)
   - Add "Copy answer" functionality
3. Add loading spinner during query processing
   - Implement visual indicator during API calls
   - Show estimated time or progress if possible
   - Disable submit button during processing to prevent duplicate requests
4. Create error message display areas
   - Design distinct styling for different error types
   - Include appropriate icons and color coding
   - Provide actionable suggestions for users
5. Implement responsive design
   - Ensure components work well on mobile devices
   - Adapt layout for different screen sizes
   - Maintain accessibility standards
6. Add keyboard navigation support
   - Enable tab navigation between components
   - Support Enter key for submission
   - Implement proper focus management

### Step 6: State Management
1. Implement loading state management
   - Track API request status (idle, loading, success, error)
   - Manage individual loading states for multiple concurrent requests
   - Update UI components based on loading state
   - Handle cancellation of long-running requests
2. Add error state tracking
   - Distinguish between different error types (network, backend, validation)
   - Store error messages and details for display
   - Implement error state transitions and resets
   - Track error frequency for potential service degradation detection
3. Create success state handling
   - Manage response data and metadata
   - Handle state updates for new responses
   - Implement state persistence across component re-renders
4. Implement query history management
   - Store recent queries and responses in component state
   - Implement history size limits (max 10 items)
   - Add history navigation functionality
   - Include timestamp tracking for each query
5. Implement global state management
   - Use context or state management library if needed for complex state
   - Share state between related components
   - Handle state persistence across page navigation

### Step 7: Integration and Testing
1. Integrate components into Docusaurus layout
   - Add RAG interface to Docusaurus sidebar or as floating panel
   - Ensure proper styling matches existing Docusaurus theme
   - Integrate with Docusaurus lifecycle hooks for proper initialization
   - Test compatibility with Docusaurus navigation and routing
2. Test full-book question flow
   - Verify end-to-end functionality from query submission to response display
   - Test with various question types and complexity levels
   - Validate source reference accuracy and formatting
   - Confirm response quality and relevance
3. Test selected-text question flow
   - Verify text selection capture functionality
   - Test context restriction effectiveness
   - Validate that answers are based on selected text
   - Test edge cases with very short or very long selections
4. Validate error handling scenarios
   - Test network error conditions (offline, timeout)
   - Verify backend error handling (500 errors, service unavailable)
   - Test validation error responses (bad requests)
   - Validate empty response handling with appropriate user messaging
5. Test performance and latency requirements
   - Measure end-to-end latency for various query types
   - Validate 90% of queries complete under 2 seconds in local setup
   - Test concurrent query handling and performance
   - Verify memory usage during extended usage sessions
6. Conduct cross-browser compatibility testing
   - Test in Chrome, Firefox, Safari, and Edge
   - Verify responsive behavior on different screen sizes
   - Validate accessibility features and keyboard navigation
7. Perform security testing
   - Verify XSS protection in rendered content
   - Test input sanitization effectiveness
   - Validate proper error message sanitization

## Dependencies

### Frontend Dependencies
- `axios` or `fetch` for HTTP requests
- React components (if using React-based Docusaurus theme)
- CSS/styling framework for UI components

### Backend Dependencies
- FastAPI RAG agent service running and accessible (default port: 8009)
- Proper CORS configuration for cross-origin requests
- Valid API keys configured in backend
- Qdrant Cloud connection for vector database

## Risk Assessment

### Technical Risks
1. **CORS Issues**: Backend may not have proper CORS headers
   - Mitigation: Ensure backend includes proper CORS configuration
2. **Network Latency**: End-to-end latency may exceed 2-second requirement
   - Mitigation: Implement caching and optimize request handling
3. **Security Vulnerabilities**: XSS from rendered content
   - Mitigation: Proper input sanitization and output encoding

### Implementation Risks
1. **Docusaurus Integration**: Theme customization may conflict with updates
   - Mitigation: Use plugin-based approach where possible
2. **Browser Compatibility**: Features may not work across all browsers
   - Mitigation: Test across target browser matrix

## Success Criteria

### Functional Requirements
- [ ] Frontend successfully sends queries and receives responses from backend
- [ ] Selected-text queries properly restrict answers to provided context
- [ ] Backend responses render correctly in UI without page refresh
- [ ] End-to-end query processing completes in under 2 seconds 90% of the time
- [ ] All failure scenarios display appropriate error messages

### Non-Functional Requirements
- [ ] System supports up to 10 queries per user session
- [ ] UI remains responsive during query processing
- [ ] Error states are clearly communicated to users
- [ ] Integration maintains Docusaurus framework agnosticism

## Testing Strategy

### Unit Tests
- API client module functionality (request formatting, response handling)
- Response parsing and formatting functions
- Error handling logic and message formatting
- Input validation functions (query length, character limits)
- Content sanitization utilities
- Cache management functions
- State management utilities

### Integration Tests
- End-to-end query flow from UI to backend and back
- Full-book vs selected-text query handling differences
- Error scenario handling across all API endpoints
- Loading state management during various request states
- Cross-component communication and state sharing
- Environment variable configuration and fallbacks

### Performance Tests
- End-to-end latency measurement for various query types
- Multiple concurrent query handling and queuing
- Response rendering performance with large answers
- Memory usage during extended sessions
- Cache performance and hit rates
- UI responsiveness during API calls

### User Acceptance Tests
- Full-book question accuracy and source reference validity
- Selected-text query context restriction effectiveness
- Error message clarity and user guidance
- UI component responsiveness and accessibility
- Cross-browser functionality validation
- Mobile device compatibility and touch interactions

### Load Tests
- Concurrent user sessions (simulated)
- Maximum query rate handling
- Backend resource utilization monitoring
- Timeout and retry mechanism effectiveness

## Deployment Considerations

### Environment Variables
- Development: Local backend endpoint
- Staging: Staging backend endpoint
- Production: Production backend endpoint

### Security
- No sensitive data in frontend code
- Proper input sanitization
- Secure communication via HTTPS

## Rollback Plan

If issues arise after deployment:
1. Disable RAG integration feature via configuration
2. Revert to static content-only mode
3. Restore previous stable version if needed