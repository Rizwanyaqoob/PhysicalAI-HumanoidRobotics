/**
 * Content Sanitization Utilities for RAG Integration
 * Prevents XSS attacks by sanitizing user input and backend responses
 */

// Regular expressions for various sanitization tasks
const SCRIPT_REGEX = /<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi;
const ON_EVENT_REGEX = /\s*on\w+\s*=\s*["'][^"']*["']/gi;
const JAVASCRIPT_PROTOCOL_REGEX = /javascript:/gi;
const DATA_PROTOCOL_REGEX = /data:/gi;
const VBSCRIPT_PROTOCOL_REGEX = /vbscript:/gi;
const EVAL_REGEX = /\beval\s*\(/gi;
const ALERT_REGEX = /\balert\s*\(/gi;
const DOCUMENT_COOKIE_REGEX = /document\.cookie/gi;
const WINDOW_LOCATION_REGEX = /window\.location/gi;

// Define TypeScript interfaces
interface SourceObject {
  content?: string;
  source_document?: string;
  page_number?: number;
  section_title?: string;
  similarity_score?: number;
  chunk_id?: string;
  [key: string]: any;
}

interface ResponseObject {
  answer?: string;
  sources?: SourceObject[];
  provider_used?: string;
  debug_info?: any;
  [key: string]: any;
}

/**
 * Sanitize HTML content to prevent XSS
 * @param html - HTML content to sanitize
 * @returns Sanitized HTML content
 */
export function sanitizeHTML(html: string): string {
  if (typeof html !== 'string') {
    return '';
  }

  let sanitized = html;

  // Remove script tags and their content
  sanitized = sanitized.replace(SCRIPT_REGEX, '');

  // Remove event handlers
  sanitized = sanitized.replace(ON_EVENT_REGEX, '');

  // Remove dangerous protocols
  sanitized = sanitized.replace(JAVASCRIPT_PROTOCOL_REGEX, 'javascript-disabled:');
  sanitized = sanitized.replace(DATA_PROTOCOL_REGEX, 'data-disabled:');
  sanitized = sanitized.replace(VBSCRIPT_PROTOCOL_REGEX, 'vbscript-disabled:');

  // Remove dangerous functions
  sanitized = sanitized.replace(EVAL_REGEX, 'eval-disabled(');
  sanitized = sanitized.replace(ALERT_REGEX, 'alert-disabled(');

  // Remove dangerous properties
  sanitized = sanitized.replace(DOCUMENT_COOKIE_REGEX, 'document-cookie-disabled');
  sanitized = sanitized.replace(WINDOW_LOCATION_REGEX, 'window-location-disabled');

  return sanitized;
}

/**
 * Escape HTML special characters
 * @param str - String to escape
 * @returns Escaped string
 */
export function escapeHTML(str: string): string {
  if (typeof str !== 'string') {
    return '';
  }

  const escapeMap: { [key: string]: string } = {
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
    '"': '&quot;',
    "'": '&#x27;',
    '/': '&#x2F;',
    '`': '&#x60;',
    '=': '&#x3D;'
  };

  return str.replace(/[&<>"'`=\/]/g, function (s) {
    return escapeMap[s];
  });
}

/**
 * Sanitize a text string
 * @param text - Text to sanitize
 * @returns Sanitized text
 */
export function sanitizeText(text: string): string {
  if (typeof text !== 'string') {
    return '';
  }

  // First escape HTML characters
  let sanitized = escapeHTML(text);

  // Remove any potential script content
  sanitized = sanitized.replace(/<script/gi, '&lt;script');
  sanitized = sanitized.replace(/javascript:/gi, 'javascript-disabled:');

  return sanitized;
}

/**
 * Sanitize URL
 * @param url - URL to sanitize
 * @returns Sanitized URL
 */
export function sanitizeURL(url: string): string {
  if (typeof url !== 'string') {
    return '';
  }

  // Only allow safe protocols
  if (url.toLowerCase().startsWith('javascript:') ||
      url.toLowerCase().startsWith('vbscript:') ||
      url.toLowerCase().startsWith('data:')) {
    return '#';
  }

  return url;
}

/**
 * Sanitize source reference data
 * @param source - Source object to sanitize
 * @returns Sanitized source object
 */
export function sanitizeSource(source: SourceObject): SourceObject {
  if (!source || typeof source !== 'object') {
    return {};
  }

  return {
    content: sanitizeHTML(source.content || ''),
    source_document: sanitizeText(source.source_document || ''),
    page_number: source.page_number, // Numbers don't need sanitization
    section_title: sanitizeText(source.section_title || ''),
    similarity_score: source.similarity_score, // Numbers don't need sanitization
    chunk_id: sanitizeText(source.chunk_id || '')
  };
}

/**
 * Sanitize an array of source references
 * @param sources - Array of source objects to sanitize
 * @returns Sanitized array of source objects
 */
export function sanitizeSources(sources: SourceObject[]): SourceObject[] {
  if (!Array.isArray(sources)) {
    return [];
  }

  return sources.map(source => sanitizeSource(source));
}

/**
 * Sanitize a complete response object
 * @param response - Response object to sanitize
 * @returns Sanitized response object
 */
export function sanitizeResponse(response: ResponseObject): ResponseObject {
  if (!response || typeof response !== 'object') {
    return {};
  }

  return {
    answer: sanitizeHTML(response.answer || ''),
    sources: sanitizeSources(response.sources || []),
    provider_used: response.provider_used || 'unknown',
    debug_info: response.debug_info || {}
  };
}

/**
 * Sanitize query input
 * @param query - Query string to sanitize
 * @returns Sanitized query
 */
export function sanitizeQuery(query: string): string {
  if (typeof query !== 'string') {
    return '';
  }

  // Limit length to prevent abuse
  if (query.length > 10000) {
    query = query.substring(0, 10000);
  }

  // Sanitize the query text
  const sanitized = sanitizeText(query);

  return sanitized;
}

/**
 * Validate and sanitize selected text
 * @param selectedText - Selected text to sanitize
 * @returns Sanitized selected text
 */
export function sanitizeSelectedText(selectedText: string): string {
  if (typeof selectedText !== 'string') {
    return '';
  }

  // Limit length to prevent abuse
  if (selectedText.length > 5000) {
    selectedText = selectedText.substring(0, 5000);
  }

  // Sanitize the selected text
  const sanitized = sanitizeText(selectedText);

  return sanitized;
}

/**
 * Comprehensive sanitizer that handles multiple types of content
 * @param content - Content to sanitize
 * @param type - Type of content ('text', 'html', 'url', 'source', 'response', 'query')
 * @returns Sanitized content
 */
export function comprehensiveSanitize(content: any, type: string = 'text'): any {
  switch (type) {
    case 'html':
      return sanitizeHTML(content);
    case 'url':
      return sanitizeURL(content);
    case 'source':
      return sanitizeSource(content);
    case 'sources':
      return sanitizeSources(content);
    case 'response':
      return sanitizeResponse(content);
    case 'query':
      return sanitizeQuery(content);
    case 'selected-text':
      return sanitizeSelectedText(content);
    case 'text':
    default:
      return sanitizeText(content);
  }
}

/**
 * Check if content contains potentially dangerous elements
 * @param content - Content to check
 * @returns True if content contains dangerous elements
 */
export function hasDangerousContent(content: string): boolean {
  if (typeof content !== 'string') {
    return false;
  }

  return (
    SCRIPT_REGEX.test(content) ||
    ON_EVENT_REGEX.test(content) ||
    JAVASCRIPT_PROTOCOL_REGEX.test(content) ||
    EVAL_REGEX.test(content) ||
    ALERT_REGEX.test(content)
  );
}

export default {
  sanitizeHTML,
  escapeHTML,
  sanitizeText,
  sanitizeURL,
  sanitizeSource,
  sanitizeSources,
  sanitizeResponse,
  sanitizeQuery,
  sanitizeSelectedText,
  comprehensiveSanitize,
  hasDangerousContent
};