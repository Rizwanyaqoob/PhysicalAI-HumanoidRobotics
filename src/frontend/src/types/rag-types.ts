/**
 * Type definitions for RAG Integration
 */

// Source object type
export interface SourceObject {
  content: string;
  source_document?: string;
  sourceDocument?: string;
  page_number?: number | null;
  pageNumber?: number | null;
  section_title?: string;
  sectionTitle?: string;
  similarity_score?: number;
  similarityScore?: number;
  chunk_id?: string;
  chunkId?: string;
}

// Debug info type
export interface DebugInfo {
  embedding_time?: number;
  embeddingTime?: number;
  retrieval_time?: number;
  retrievalTime?: number;
  generation_time?: number;
  generationTime?: number;
  total_time?: number;
  totalTime?: number;
}

// Response object type
export interface ResponseObject {
  answer?: string;
  sources?: SourceObject[];
  debug_info?: DebugInfo;
  debugInfo?: DebugInfo;
  provider_used?: string;
  providerUsed?: string;
  [key: string]: any;
}

// Parsed response type
export interface ParsedResponse {
  answer: string;
  sources: SourceObject[];
  timingInfo: {
    embeddingTime: number;
    retrievalTime: number;
    generationTime: number;
    totalTime: number;
  };
  providerUsed: string;
  isValidForSelectedText: boolean | null;
}

// Source for display type
export interface SourceForDisplay {
  id: string;
  content: string;
  sourceDocument: string;
  pageNumber: number | null;
  sectionTitle: string;
  similarityScore: number;
  displayText: string;
}

// Response stats type
export interface ResponseStats {
  sourceCount: number;
  avgSimilarity: number;
  totalProcessingTime: number;
}

// Validation result type
export interface ValidationResult {
  isValid: boolean;
  error: string | null;
}

// Validation request result type
export interface ValidationRequestResult {
  isValid: boolean;
  errors: string[];
}

// Sanitization result type
export interface SanitizationResult {
  sanitized: string;
  isValid: boolean;
  error: string | null;
}

// History item type
export interface HistoryItem {
  id?: string;
  query: string;
  response?: {
    answer?: string;
    [key: string]: any;
  };
  date: string;
  timestamp?: string;
  [key: string]: any;
}

// Selection info type
export interface SelectionInfo {
  text: string;
  startOffset: number | null;
  endOffset: number | null;
  startContainer: Node | null;
  endContainer: Node | null;
  range: Range | null;
}

// Query options type
export interface QueryOptions {
  [key: string]: any;
}

// Cache entry type
export interface CacheEntry<T = any> {
  value: T;
  expiration: number;
}

// Cache stats type
export interface CacheStats {
  size: number;
  maxCacheSize: number;
  defaultTTL: number;
}

// State stats type
export interface StateStats {
  historySize: number;
  maxHistorySize: number;
  loadingOperations: number;
  errorOperations: number;
}

// Error state type
export interface ErrorState {
  error: any;
  timestamp: string;
}

// Metric item type
export interface MetricItem {
  value: number;
  timestamp: string;
}

// Performance metrics type
export interface PerformanceMetrics {
  queryTimes: MetricItem[];
  apiCallTimes: MetricItem[];
  renderTimes: MetricItem[];
  cacheHitRates: MetricItem[];
  errorRates: MetricItem[];
}

// Metric stats type
export interface MetricStats {
  count: number;
  average: number;
  min: number;
  max: number;
  last: number;
}

// Query time compliance type
export interface QueryTimeCompliance {
  percentage: number;
  threshold: number;
  compliant: boolean;
}

// Performance stats type
export interface PerformanceStats {
  [key: string]: MetricStats | QueryTimeCompliance;
}

// Request options type
export interface RequestOptions {
  method?: string;
  headers?: Record<string, string>;
  body?: string;
  [key: string]: any;
}

// API response type
export interface ApiResponse {
  ok: boolean;
  status: number;
  statusText: string;
  json: () => Promise<any>;
  text: () => Promise<string>;
  [key: string]: any;
}

// Error response type
export interface ErrorResponse {
  response?: {
    status: number;
    statusText: string;
    data: { message: string };
  };
  request?: any;
  message: string;
}

// Queue item type
export interface QueueItem {
  requestFn: () => Promise<any>;
  resolve: (value: any) => void;
  reject: (reason: any) => void;
}

// Error display props type
export type ErrorType = 'network' | 'backend' | 'validation' | 'empty' | 'general';

// Error display props type
export interface ErrorDisplayProps {
  error?: string | object | null;
  showError?: boolean;
  onRetry?: () => void | null;
  showRetryButton?: boolean;
  errorType?: ErrorType;
}

// Response display props type
export interface ResponseDisplayProps {
  response?: ResponseObject;
  show?: boolean;
}

// Loading indicator props type
export interface LoadingIndicatorProps {
  show?: boolean;
  message?: string;
  showProgress?: boolean;
  estimatedTime?: number | null;
}

// Submit button props type
export interface SubmitButtonProps {
  onClick?: (e: React.MouseEvent<HTMLButtonElement>) => void;
  disabled?: boolean;
  text?: string;
  loading?: boolean;
  className?: string;
}

// Query input props type
export interface QueryInputProps {
  onQuerySubmit: (query: string, selectedText?: string | null) => void;
  selectedText?: string | null;
  onSelectedTextChange?: (selectedText: string | null) => void;
}