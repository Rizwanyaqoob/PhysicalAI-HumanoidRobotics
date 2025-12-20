/**
 * Cache Service for RAG Integration
 * Implements in-memory caching for repeated queries with expiration
 */

// Define TypeScript interfaces
interface CacheEntry<T = any> {
  value: T;
  expiration: number;
}

interface CacheStats {
  size: number;
  maxCacheSize: number;
  defaultTTL: number;
}

interface QueryOptions {
  [key: string]: any;
}

class CacheService {
  private cache: Map<string, CacheEntry>;
  private maxCacheSize: number;
  private defaultTTL: number;

  constructor() {
    this.cache = new Map();
    this.maxCacheSize = 50; // Maximum number of items to store
    this.defaultTTL = 10 * 60 * 1000; // 10 minutes in milliseconds
  }

  /**
   * Generate a cache key based on query and options
   * @param query - The query string
   * @param options - Query options (provider, maxChunks, etc.)
   * @returns Cache key
   */
  generateKey(query: string, options: QueryOptions = {}): string {
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
   * @param key - Cache key
   * @param value - Value to store
   * @param ttl - Time to live in milliseconds
   */
  set<T = any>(key: string, value: T, ttl: number = this.defaultTTL): void {
    // Check if cache is at max size and remove oldest items
    if (this.cache.size >= this.maxCacheSize) {
      const firstKey = this.cache.keys().next().value;
      if (firstKey) {
        this.cache.delete(firstKey);
      }
    }

    const expiration = Date.now() + ttl;
    this.cache.set(key, { value, expiration });
  }

  /**
   * Get a value from cache
   * @param key - Cache key
   * @returns Cached value or undefined if not found or expired
   */
  get<T = any>(key: string): T | undefined {
    const cached = this.cache.get(key);

    if (!cached) {
      return undefined;
    }

    // Check if the item has expired
    if (Date.now() > cached.expiration) {
      this.cache.delete(key);
      return undefined;
    }

    return cached.value as T;
  }

  /**
   * Check if a key exists in cache and is not expired
   * @param key - Cache key
   * @returns True if key exists and is not expired
   */
  has(key: string): boolean {
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
  cleanup(): void {
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
  clear(): void {
    this.cache.clear();
  }

  /**
   * Get cache statistics
   * @returns Cache statistics
   */
  getStats(): CacheStats {
    this.cleanup(); // Clean up expired items first
    return {
      size: this.cache.size,
      maxCacheSize: this.maxCacheSize,
      defaultTTL: this.defaultTTL
    };
  }

  /**
   * Cache a query response
   * @param query - The query string
   * @param options - Query options
   * @param response - The response to cache
   * @param ttl - Time to live in milliseconds
   */
  cacheQuery(query: string, options: QueryOptions, response: any, ttl: number = this.defaultTTL): void {
    const key = this.generateKey(query, options);
    this.set(key, response, ttl);
  }

  /**
   * Get cached response for a query
   * @param query - The query string
   * @param options - Query options
   * @returns Cached response or undefined
   */
  getCachedQuery(query: string, options: QueryOptions): any {
    const key = this.generateKey(query, options);
    return this.get(key);
  }
}

// Create and export a singleton instance
export const cacheService = new CacheService();

// Export the CacheService class as default
export default CacheService;