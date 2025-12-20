/**
 * Cache Service for RAG Integration
 * Implements in-memory caching for repeated queries with expiration
 */

class CacheService {
  constructor() {
    this.cache = new Map();
    this.maxCacheSize = 50; // Maximum number of items to store
    this.defaultTTL = 10 * 60 * 1000; // 10 minutes in milliseconds
  }

  /**
   * Generate a cache key based on query and options
   * @param {string} query - The query string
   * @param {Object} options - Query options (provider, maxChunks, etc.)
   * @returns {string} Cache key
   */
  generateKey(query, options = {}) {
    const keyString = JSON.stringify({
      query: query,
      options: options
    });
    // Simple hash function to create a consistent key
    let hash = 0;
    for (let i = 0; i < keyString.length; i++) {
      const char = keyString.charCodeAt(i);
      hash = ((hash << 5) - hash) + char;
      hash = hash & hash; // Convert to 32-bit integer
    }
    return hash.toString();
  }

  /**
   * Store a value in cache with TTL
   * @param {string} key - Cache key
   * @param {*} value - Value to store
   * @param {number} ttl - Time to live in milliseconds
   */
  set(key, value, ttl = this.defaultTTL) {
    // Check if cache is at max size and remove oldest items
    if (this.cache.size >= this.maxCacheSize) {
      const firstKey = this.cache.keys().next().value;
      this.cache.delete(firstKey);
    }

    const expiration = Date.now() + ttl;
    this.cache.set(key, { value, expiration });
  }

  /**
   * Get a value from cache
   * @param {string} key - Cache key
   * @returns {*} Cached value or undefined if not found or expired
   */
  get(key) {
    const cached = this.cache.get(key);

    if (!cached) {
      return undefined;
    }

    // Check if the item has expired
    if (Date.now() > cached.expiration) {
      this.cache.delete(key);
      return undefined;
    }

    return cached.value;
  }

  /**
   * Check if a key exists in cache and is not expired
   * @param {string} key - Cache key
   * @returns {boolean} True if key exists and is not expired
   */
  has(key) {
    const cached = this.cache.get(key);

    if (!cached) {
      return false;
    }

    // Check if the item has expired
    if (Date.now() > cached.expiration) {
      this.cache.delete(key);
      return false;
    }

    return true;
  }

  /**
   * Clear expired items from cache
   */
  cleanup() {
    const now = Date.now();
    for (const [key, { expiration }] of this.cache.entries()) {
      if (now > expiration) {
        this.cache.delete(key);
      }
    }
  }

  /**
   * Clear all items from cache
   */
  clear() {
    this.cache.clear();
  }

  /**
   * Get cache statistics
   * @returns {Object} Cache statistics
   */
  getStats() {
    this.cleanup(); // Clean up expired items first
    return {
      size: this.cache.size,
      maxCacheSize: this.maxCacheSize,
      defaultTTL: this.defaultTTL
    };
  }

  /**
   * Cache a query response
   * @param {string} query - The query string
   * @param {Object} options - Query options
   * @param {*} response - The response to cache
   * @param {number} ttl - Time to live in milliseconds
   */
  cacheQuery(query, options, response, ttl = this.defaultTTL) {
    const key = this.generateKey(query, options);
    this.set(key, response, ttl);
  }

  /**
   * Get cached response for a query
   * @param {string} query - The query string
   * @param {Object} options - Query options
   * @returns {*} Cached response or undefined
   */
  getCachedQuery(query, options) {
    const key = this.generateKey(query, options);
    return this.get(key);
  }
}

// Create and export a singleton instance
export const cacheService = new CacheService();

// Export the CacheService class as default
export default CacheService;