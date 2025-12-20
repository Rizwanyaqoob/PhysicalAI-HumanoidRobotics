/**
 * State Manager for RAG Integration
 * Manages query history, loading states, and other UI states
 */

class StateManager {
  constructor() {
    this.maxHistorySize = 10; // Maximum number of items in history
    this.queryHistory = [];
    this.loadingStates = new Map(); // Track loading states for different operations
    this.errorStates = new Map(); // Track error states for different operations
  }

  /**
   * Add a query to history
   * @param {string} query - The query string
   * @param {Object} response - The response object
   * @param {string} timestamp - Optional timestamp (will be generated if not provided)
   */
  addQueryToHistory(query, response, timestamp = null) {
    const historyItem = {
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
   * @returns {Array} Array of query history items
   */
  getQueryHistory() {
    return [...this.queryHistory]; // Return a copy to prevent direct mutation
  }

  /**
   * Clear query history
   */
  clearQueryHistory() {
    this.queryHistory = [];
  }

  /**
   * Set loading state for an operation
   * @param {string} operationId - Unique identifier for the operation
   * @param {boolean} isLoading - Whether the operation is loading
   */
  setLoadingState(operationId, isLoading) {
    this.loadingStates.set(operationId, isLoading);
  }

  /**
   * Get loading state for an operation
   * @param {string} operationId - Unique identifier for the operation
   * @returns {boolean} Whether the operation is loading
   */
  getLoadingState(operationId) {
    return this.loadingStates.get(operationId) || false;
  }

  /**
   * Set error state for an operation
   * @param {string} operationId - Unique identifier for the operation
   * @param {Object|string} error - The error object or message
   */
  setErrorState(operationId, error) {
    this.errorStates.set(operationId, {
      error: error,
      timestamp: new Date().toISOString()
    });
  }

  /**
   * Get error state for an operation
   * @param {string} operationId - Unique identifier for the operation
   * @returns {Object} Error state object with error and timestamp
   */
  getErrorState(operationId) {
    return this.errorStates.get(operationId) || null;
  }

  /**
   * Clear error state for an operation
   * @param {string} operationId - Unique identifier for the operation
   */
  clearErrorState(operationId) {
    this.errorStates.delete(operationId);
  }

  /**
   * Clear all states
   */
  clearAllStates() {
    this.queryHistory = [];
    this.loadingStates.clear();
    this.errorStates.clear();
  }

  /**
   * Generate a unique ID
   * @returns {string} Unique ID
   */
  generateId() {
    return 'id_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
  }

  /**
   * Get state statistics
   * @returns {Object} State statistics
   */
  getStats() {
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