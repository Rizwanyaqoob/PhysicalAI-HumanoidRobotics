/**
 * Performance Monitoring for RAG Integration
 * Measures query processing times and provides performance metrics
 */

class PerformanceMonitor {
  constructor() {
    this.metrics = {
      queryTimes: [],
      apiCallTimes: [],
      renderTimes: [],
      cacheHitRates: [],
      errorRates: []
    };
    this.maxMetricsToStore = 100; // Maximum number of metrics to store
    this.performanceThreshold = 2000; // 2 seconds threshold in milliseconds
  }

  /**
   * Start timing an operation
   * @param {string} operationId - Unique identifier for the operation
   */
  startTiming(operationId) {
    if (!operationId) {
      console.error('Operation ID is required for timing');
      return;
    }
    sessionStorage.setItem(`timing_${operationId}`, Date.now().toString());
  }

  /**
   * End timing an operation and record the duration
   * @param {string} operationId - Unique identifier for the operation
   * @returns {number} Duration in milliseconds
   */
  endTiming(operationId) {
    if (!operationId) {
      console.error('Operation ID is required for timing');
      return 0;
    }

    const startTimeStr = sessionStorage.getItem(`timing_${operationId}`);
    if (!startTimeStr) {
      console.warn(`No start time found for operation: ${operationId}`);
      return 0;
    }

    const startTime = parseInt(startTimeStr);
    const endTime = Date.now();
    const duration = endTime - startTime;

    // Remove the timing entry
    sessionStorage.removeItem(`timing_${operationId}`);

    // Store the duration in the appropriate metric array
    this.recordMetric(operationId, duration);

    return duration;
  }

  /**
   * Record a performance metric
   * @param {string} metricType - Type of metric (queryTime, apiCallTime, etc.)
   * @param {number} value - Value to record
   */
  recordMetric(metricType, value) {
    if (!this.metrics[`${metricType}s`]) {
      // Create the array if it doesn't exist
      this.metrics[`${metricType}s`] = [];
    }

    // Add the value to the appropriate array
    const metricArray = this.metrics[`${metricType}s`];
    metricArray.push({
      value: value,
      timestamp: new Date().toISOString()
    });

    // Keep only the most recent metrics up to maxMetricsToStore
    if (metricArray.length > this.maxMetricsToStore) {
      this.metrics[`${metricType}s`] = metricArray.slice(-this.maxMetricsToStore);
    }
  }

  /**
   * Check if a query time is within the performance threshold (90% under 2 seconds)
   * @param {number} queryTime - Query time in milliseconds
   * @returns {boolean} True if the query time is acceptable
   */
  isQueryTimeAcceptable(queryTime) {
    // Add the new query time to our metrics
    this.recordMetric('queryTime', queryTime);

    // Calculate the percentage of queries under the threshold
    const recentQueryTimes = this.metrics.queryTimes.slice(-50); // Look at last 50 queries
    if (recentQueryTimes.length === 0) {
      return true; // If no data, assume acceptable
    }

    const underThresholdCount = recentQueryTimes.filter(
      item => item.value <= this.performanceThreshold
    ).length;

    const percentageUnderThreshold = (underThresholdCount / recentQueryTimes.length) * 100;

    return percentageUnderThreshold >= 90;
  }

  /**
   * Get average query time
   * @returns {number} Average query time in milliseconds
   */
  getAverageQueryTime() {
    if (this.metrics.queryTimes.length === 0) {
      return 0;
    }

    const sum = this.metrics.queryTimes.reduce((acc, item) => acc + item.value, 0);
    return sum / this.metrics.queryTimes.length;
  }

  /**
   * Get performance statistics
   * @returns {Object} Performance statistics
   */
  getStats() {
    const stats = {};

    for (const [metricType, metricArray] of Object.entries(this.metrics)) {
      if (metricArray.length > 0) {
        const values = metricArray.map(item => item.value);
        const sum = values.reduce((acc, val) => acc + val, 0);

        stats[metricType] = {
          count: values.length,
          average: sum / values.length,
          min: Math.min(...values),
          max: Math.max(...values),
          last: values[values.length - 1]
        };
      } else {
        stats[metricType] = {
          count: 0,
          average: 0,
          min: 0,
          max: 0,
          last: 0
        };
      }
    }

    // Add performance threshold compliance
    const recentQueryTimes = this.metrics.queryTimes.slice(-50);
    if (recentQueryTimes.length > 0) {
      const underThresholdCount = recentQueryTimes.filter(
        item => item.value <= this.performanceThreshold
      ).length;
      stats.queryTimeCompliance = {
        percentage: (underThresholdCount / recentQueryTimes.length) * 100,
        threshold: this.performanceThreshold,
        compliant: (underThresholdCount / recentQueryTimes.length) >= 0.9
      };
    }

    return stats;
  }

  /**
   * Reset all performance metrics
   */
  reset() {
    this.metrics = {
      queryTimes: [],
      apiCallTimes: [],
      renderTimes: [],
      cacheHitRates: [],
      errorRates: []
    };
  }

  /**
   * Log performance metrics to console
   */
  logMetrics() {
    const stats = this.getStats();
    console.group('RAG Integration Performance Metrics');
    console.table(stats);
    console.groupEnd();
  }
}

// Create and export a singleton instance
export const performanceMonitor = new PerformanceMonitor();

// Export the PerformanceMonitor class as default
export default PerformanceMonitor;