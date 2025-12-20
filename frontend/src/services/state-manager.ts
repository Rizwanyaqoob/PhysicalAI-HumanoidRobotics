/**
 * State Manager for RAG Integration
 * Manages query history, loading states, and other UI states
 */

// Define TypeScript interfaces
interface HistoryItem {
  id: string;
  query: string;
  response: any;
  timestamp: string;
  date: string;
}

interface ErrorState {
  error: any;
  timestamp: string;
}

interface StateStats {
  historySize: number;
  maxHistorySize: number;
  loadingOperations: number;
  errorOperations: number;
}

class StateManager {
  private maxHistorySize: number;
  private queryHistory: HistoryItem[];
  private loadingStates: Map<string, boolean>; // Track loading states for different operations
  private errorStates: Map<string, ErrorState>; // Track error states for different operations

  constructor() {
    this.maxHistorySize = 10; // Maximum number of items in history
    this.queryHistory = [];
    this.loadingStates = new Map(); // Track loading states for different operations
    this.errorStates = new Map(); // Track error states for different operations
  }

  /**
   * Add a query to history
   * @param query - The query string
   * @param response - The response object
   * @param timestamp - Optional timestamp (will be generated if not provided)
   */
  addQueryToHistory(query: string, response: any, timestamp: string | null = null): void {
    const historyItem: HistoryItem = {
      id: this.generateId(),
      query: query,
      response: response,
      timestamp: timestamp || new Date().toISOString(),
      date: new Date().toLocaleString()
    };

    // Add to the beginning of the array (most recent first)
    this.queryHistory.unshift(historyItem);

    // Keep only the most recent items up to maxHistorySize
    if (this.queryHistory.length > this.maxHistorySize) {
      this.queryHistory = this.queryHistory.slice(0, this.maxHistorySize);
    }
  }

  /**
   * Get query history
   * @returns Array of query history items
   */
  getQueryHistory(): HistoryItem[] {
    return [...this.queryHistory]; // Return a copy to prevent direct mutation
  }

  /**
   * Clear query history
   */
  clearQueryHistory(): void {
    this.queryHistory = [];
  }

  /**
   * Set loading state for an operation
   * @param operationId - Unique identifier for the operation
   * @param isLoading - Whether the operation is loading
   */
  setLoadingState(operationId: string, isLoading: boolean): void {
    this.loadingStates.set(operationId, isLoading);
  }

  /**
   * Get loading state for an operation
   * @param operationId - Unique identifier for the operation
   * @returns Whether the operation is loading
   */
  getLoadingState(operationId: string): boolean {
    return this.loadingStates.get(operationId) || false;
  }

  /**
   * Set error state for an operation
   * @param operationId - Unique identifier for the operation
   * @param error - The error object or message
   */
  setErrorState(operationId: string, error: any): void {
    this.errorStates.set(operationId, {
      error: error,
      timestamp: new Date().toISOString()
    });
  }

  /**
   * Get error state for an operation
   * @param operationId - Unique identifier for the operation
   * @returns Error state object with error and timestamp
   */
  getErrorState(operationId: string): ErrorState | null {
    return this.errorStates.get(operationId) || null;
  }

  /**
   * Clear error state for an operation
   * @param operationId - Unique identifier for the operation
   */
  clearErrorState(operationId: string): void {
    this.errorStates.delete(operationId);
  }

  /**
   * Clear all states
   */
  clearAllStates(): void {
    this.queryHistory = [];
    this.loadingStates.clear();
    this.errorStates.clear();
  }

  /**
   * Generate a unique ID
   * @returns Unique ID
   */
  generateId(): string {
    return 'id_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
  }

  /**
   * Get state statistics
   * @returns State statistics
   */
  getStats(): StateStats {
    return {
      historySize: this.queryHistory.length,
      maxHistorySize: this.maxHistorySize,
      loadingOperations: Array.from(this.loadingStates.entries()).filter(([_, isLoading]) => isLoading).length,
      errorOperations: this.errorStates.size
    };
  }
}

// Create and export a singleton instance
export const stateManager = new StateManager();

// Export the StateManager class as default
export default StateManager;