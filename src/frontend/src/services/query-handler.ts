import { apiClient } from './api';
import { sanitizeQuery } from '../utils/sanitizer';
import { parseResponse } from '../utils/response-parser';

// Define TypeScript interfaces
interface ParsedResponse {
  answer: string;
  sources: Array<any>;
  timingInfo: any;
  providerUsed: string;
  isValidForSelectedText: boolean | null;
}

interface ValidationResult {
  isValid: boolean;
  errors: string[];
}

interface QueueItem {
  requestFn: () => Promise<any>;
  resolve: (value: any) => void;
  reject: (reason: any) => void;
}

// Maximum number of queries allowed per session
const MAX_QUERIES_PER_SESSION = 10;

// Request queue to handle multiple concurrent requests
let requestQueue: QueueItem[] = [];
let isProcessing = false;

/**
 * Send a query to the backend RAG agent for full-book questions
 * @param query - The user's question
 * @param provider - The AI provider to use (default: 'gemini')
 * @param maxChunks - Maximum number of content chunks to retrieve (default: 5)
 * @returns The parsed response from the backend
 */
export async function sendQuery(query: string, provider: string = 'gemini', maxChunks: number = 5): Promise<ParsedResponse> {
  // Sanitize the query
  const sanitizedQuery = sanitizeQuery(query);

  // Validate query length
  if (sanitizedQuery.length === 0) {
    throw new Error('Query cannot be empty');
  }

  if (sanitizedQuery.length > 10000) {
    throw new Error('Query exceeds maximum length of 10,000 characters');
  }

  try {
    // Send the query to the backend
    const response = await apiClient.ask(sanitizedQuery, provider, maxChunks);

    // Parse and return the response
    return parseResponse(response);
  } catch (error) {
    console.error('Error sending query:', error);
    throw error;
  }
}

/**
 * Send a query with selected text context
 * @param query - The user's question
 * @param selectedText - The selected text to provide context
 * @param provider - The AI provider to use (default: 'gemini')
 * @param maxChunks - Maximum number of content chunks to retrieve (default: 5)
 * @returns The parsed response from the backend
 */
export async function sendSelectedTextQuery(query: string, selectedText: string, provider: string = 'gemini', maxChunks: number = 5): Promise<ParsedResponse> {
  // Sanitize inputs
  const sanitizedQuery = sanitizeQuery(query);
  const sanitizedSelectedText = sanitizeQuery(selectedText);

  // Validate inputs
  if (sanitizedQuery.length === 0) {
    throw new Error('Query cannot be empty');
  }

  if (sanitizedSelectedText.length === 0) {
    throw new Error('Selected text cannot be empty for context-restricted query');
  }

  // Create a contextual query by prepending the selected text
  const contextualQuery = `Based on the following text: "${sanitizedSelectedText}", ${sanitizedQuery}`;

  try {
    // Send the contextual query to the backend
    const response = await apiClient.ask(contextualQuery, provider, maxChunks);

    // Parse and return the response, validating it's based on selected text
    const parsedResponse = parseResponse(response, sanitizedSelectedText);

    return parsedResponse;
  } catch (error) {
    console.error('Error sending selected text query:', error);
    throw error;
  }
}

/**
 * Add a request to the queue
 * @param requestFn - The request function to execute
 * @returns Promise that resolves when the request is completed
 */
export function addToRequestQueue(requestFn: () => Promise<any>): Promise<any> {
  return new Promise((resolve, reject) => {
    requestQueue.push({ requestFn, resolve, reject });

    // Process the queue if not already processing
    if (!isProcessing) {
      processQueue();
    }
  });
}

/**
 * Process requests in the queue
 */
async function processQueue(): Promise<void> {
  if (requestQueue.length === 0) {
    isProcessing = false;
    return;
  }

  isProcessing = true;
  const { requestFn, resolve, reject } = requestQueue.shift()!;

  try {
    const result = await requestFn();
    resolve(result);
  } catch (error) {
    reject(error);
  }

  // Process the next item in the queue
  await processQueue();
}

/**
 * Check if the query limit has been reached for the session
 * @returns True if the query limit has been reached
 */
export function isQueryLimitReached(): boolean {
  // In a real implementation, this would track queries per session
  // For now, we'll just return false to allow all queries
  return false;
}

/**
 * Validate query input
 * @param query - The query to validate
 * @returns Validation result with isValid and errors
 */
export function validateQuery(query: string): ValidationResult {
  const errors: string[] = [];

  if (!query || typeof query !== 'string') {
    errors.push('Query is required and must be a string');
  } else {
    if (query.trim().length === 0) {
      errors.push('Query cannot be empty');
    }

    if (query.length > 10000) {
      errors.push('Query exceeds maximum length of 10,000 characters');
    }
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Validate selected text input
 * @param selectedText - The selected text to validate
 * @returns Validation result with isValid and errors
 */
export function validateSelectedText(selectedText: string): ValidationResult {
  const errors: string[] = [];

  if (!selectedText || typeof selectedText !== 'string') {
    errors.push('Selected text is required and must be a string');
  } else {
    if (selectedText.trim().length === 0) {
      errors.push('Selected text cannot be empty');
    }

    if (selectedText.length > 5000) {
      errors.push('Selected text exceeds maximum length of 5,000 characters');
    }
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Format query for the backend API
 * @param query - The user's query
 * @param selectedText - Optional selected text context
 * @returns Formatted query for the backend
 */
export function formatQueryForBackend(query: string, selectedText: string | null = null): string {
  if (selectedText) {
    // Prepend selected text to query as context
    return `Based on the following text: "${selectedText}", ${query}`;
  }
  return query;
}

export default {
  sendQuery,
  sendSelectedTextQuery,
  addToRequestQueue,
  isQueryLimitReached,
  validateQuery,
  validateSelectedText,
  formatQueryForBackend
};