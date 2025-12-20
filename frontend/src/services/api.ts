/**
 * API Service Module for RAG Integration
 * Handles communication with the backend RAG agent service
 */

// Define TypeScript interfaces
interface RequestOptions {
  method?: string;
  headers?: Record<string, string>;
  body?: string;
  [key: string]: any;
}

interface ApiResponse {
  ok: boolean;
  status: number;
  statusText: string;
  json: () => Promise<any>;
  text: () => Promise<string>;
  [key: string]: any;
}

interface AskRequestBody {
  query: string;
  provider: string;
  max_chunks: number;
  user_id?: string | null;
}

interface RetrieveRequestBody {
  query: string;
  max_chunks: number;
}

interface ErrorResponse {
  response?: {
    status: number;
    statusText: string;
    data: { message: string };
  };
  request?: any;
  message: string;
}

// Import environment variables
const BACKEND_API_URL: string = process.env.BACKEND_API_URL || 'http://localhost:8009';

/**
 * Request interceptor function
 */
function onRequest(config: RequestOptions): RequestOptions {
  // Add authentication headers if available
  const token = localStorage.getItem('authToken');
  if (token) {
    config.headers = {
      ...config.headers,
      'Authorization': `Bearer ${token}`
    };
  }

  // Add common headers
  config.headers = {
    ...config.headers,
    'X-Request-ID': generateRequestId(),
    'X-Timestamp': new Date().toISOString()
  };

  return config;
}

/**
 * Request error interceptor
 */
function onRequestError(error: any): Promise<never> {
  console.error('Request error:', error);
  return Promise.reject(error);
}

/**
 * Response interceptor function
 */
function onResponse(response: any): any {
  // Log successful responses
  console.log(`API call successful: ${response.url} - Status: ${response.status}`);
  return response;
}

/**
 * Response error interceptor
 */
function onResponseError(error: any): any {
  if (error.response) {
    // Server responded with error status
    const { status, data } = error.response;
    console.error(`API call failed: Status ${status}`, data);

    // Specific handling for different error types
    switch (status) {
      case 400:
        console.error('Bad Request: Check request parameters');
        break;
      case 401:
        console.error('Unauthorized: Authentication required');
        break;
      case 403:
        console.error('Forbidden: Access denied');
        break;
      case 404:
        console.error('Not Found: API endpoint not available');
        break;
      case 500:
        console.error('Internal Server Error: Server encountered an error');
        break;
      default:
        console.error(`HTTP Error: ${status}`);
    }
  } else if (error.request) {
    // Request was made but no response received (network error)
    console.error('Network Error: Unable to reach the server');
  } else {
    // Something else happened
    console.error('Error:', error.message);
  }

  return Promise.reject(error);
}

/**
 * Generate a unique request ID
 */
function generateRequestId(): string {
  return 'req_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
}

/**
 * Fetch wrapper with timeout and retry logic
 */
async function fetchWithTimeout(url: string, options: RequestOptions = {}, timeout: number = 30000): Promise<any> {
  // Apply request interceptor
  const interceptedOptions = onRequest(options);

  // Create an AbortController for timeout
  const controller = new AbortController();
  const id = setTimeout(() => controller.abort(), timeout);

  try {
    const response = await fetch(url, {
      ...interceptedOptions,
      signal: controller.signal
    });

    // Apply response interceptor
    const interceptedResponse = onResponse(response);
    clearTimeout(id);
    return interceptedResponse;
  } catch (error) {
    clearTimeout(id);

    // Apply response error interceptor
    try {
      return onResponseError({
        request: error,
        message: (error as Error).message
      });
    } catch (interceptorError) {
      throw error;
    }
  }
}

/**
 * Main API client with retry logic and error handling
 */
export class APIClient {
  private baseURL: string;

  constructor(baseURL: string = BACKEND_API_URL) {
    this.baseURL = baseURL;
  }

  /**
   * Make an API request with retry logic
   */
  async request(endpoint: string, options: RequestOptions = {}, maxRetries: number = 3): Promise<ApiResponse> {
    const url = `${this.baseURL}${endpoint}`;
    let lastError: any;

    for (let attempt = 0; attempt <= maxRetries; attempt++) {
      try {
        const response = await fetchWithTimeout(url, options, 30000);

        // If successful, return the response
        if (response.ok) {
          return response;
        }

        // If it's a client error (4xx), don't retry
        if (response.status >= 400 && response.status < 500) {
          const errorText = await response.text();
          throw new Error(`Client error: ${response.status} - ${response.statusText}. Details: ${errorText}`);
        }

        // For server errors (5xx), prepare for retry
        const errorText = await response.text();
        lastError = new Error(`Server error: ${response.status} - ${response.statusText}. Details: ${errorText}`);

        // Wait before retrying (exponential backoff)
        if (attempt < maxRetries) {
          await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
        }
      } catch (error) {
        // If it's a network error or timeout, prepare for retry
        if ((error as Error).name === 'AbortError') {
          lastError = new Error('Request timeout');
        } else {
          lastError = error;
        }

        // If it's the last attempt, throw the error
        if (attempt >= maxRetries) {
          return onResponseError({
            response: { status: 0, statusText: 'Max retries exceeded', data: { message: (lastError as Error).message } },
            message: (lastError as Error).message
          });
        }

        // Wait before retrying (exponential backoff)
        await new Promise(resolve => setTimeout(resolve, Math.pow(2, attempt) * 1000));
      }
    }

    throw lastError;
  }

  /**
   * Send a query to the backend RAG agent
   */
  async ask(query: string, provider: string = 'gemini', maxChunks: number = 5, userId: string | null = null) {
    const requestBody: AskRequestBody = {
      query: query,
      provider: provider,
      max_chunks: maxChunks
    };

    if (userId) {
      requestBody.user_id = userId;
    }

    const response = await this.request('/api/ask', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody),
    });

    return response.json();
  }

  /**
   * Retrieve relevant content chunks from the backend
   */
  async retrieve(query: string, maxChunks: number = 5) {
    const requestBody: RetrieveRequestBody = {
      query: query,
      max_chunks: maxChunks
    };

    const response = await this.request('/api/retrieve', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody),
    });

    return response.json();
  }

  /**
   * Check the health of the backend service
   */
  async health() {
    const response = await this.request('/health', {
      method: 'GET',
    });

    return response.json();
  }
}

// Create a singleton instance of the API client
export const apiClient = new APIClient();

// Export the API client instance as default
export default apiClient;