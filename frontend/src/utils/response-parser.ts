/**
 * Response Parser for RAG Integration
 * Handles parsing of backend JSON responses and extracting relevant information
 */

// Define TypeScript interfaces
interface SourceObject {
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

interface DebugInfo {
  embedding_time?: number;
  embeddingTime?: number;
  retrieval_time?: number;
  retrievalTime?: number;
  generation_time?: number;
  generationTime?: number;
  total_time?: number;
  totalTime?: number;
}

interface ResponseObject {
  answer?: string;
  sources?: SourceObject[];
  debug_info?: DebugInfo;
  debugInfo?: DebugInfo;
  provider_used?: string;
  providerUsed?: string;
  [key: string]: any;
}

interface ParsedResponse {
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

interface SourceForDisplay {
  id: string;
  content: string;
  sourceDocument: string;
  pageNumber: number | null;
  sectionTitle: string;
  similarityScore: number;
  displayText: string;
}

interface ResponseStats {
  sourceCount: number;
  avgSimilarity: number;
  totalProcessingTime: number;
}

/**
 * Parse the answer text from the response
 * @param response - The backend response object
 * @returns The parsed answer text
 */
export function parseAnswer(response: ResponseObject): string {
  if (!response || typeof response.answer !== 'string') {
    throw new Error('Invalid response format: answer field is missing or not a string');
  }
  return response.answer;
}

/**
 * Extract source references from the response
 * @param response - The backend response object
 * @returns Array of source reference objects
 */
export function extractSources(response: ResponseObject): SourceObject[] {
  if (!response || !Array.isArray(response.sources)) {
    console.warn('Invalid response format: sources field is missing or not an array');
    return [];
  }

  return response.sources.map(source => ({
    content: source.content || '',
    sourceDocument: source.source_document || source.sourceDocument || '',
    pageNumber: source.page_number || source.pageNumber || null,
    sectionTitle: source.section_title || source.sectionTitle || '',
    similarityScore: source.similarity_score || source.similarityScore || 0,
    chunkId: source.chunk_id || source.chunkId || ''
  }));
}

/**
 * Process timing information from response debug info
 * @param response - The backend response object
 * @returns Timing information object
 */
export function processTimingInfo(response: ResponseObject): {
  embeddingTime: number;
  retrievalTime: number;
  generationTime: number;
  totalTime: number
} {
  if (!response || !response.debug_info && !response.debugInfo) {
    return {
      embeddingTime: 0,
      retrievalTime: 0,
      generationTime: 0,
      totalTime: 0
    };
  }

  const debugInfo = response.debug_info || response.debugInfo;
  return {
    embeddingTime: (debugInfo as DebugInfo).embedding_time || (debugInfo as DebugInfo).embeddingTime || 0,
    retrievalTime: (debugInfo as DebugInfo).retrieval_time || (debugInfo as DebugInfo).retrievalTime || 0,
    generationTime: (debugInfo as DebugInfo).generation_time || (debugInfo as DebugInfo).generationTime || 0,
    totalTime: (debugInfo as DebugInfo).total_time || (debugInfo as DebugInfo).totalTime || 0
  };
}

/**
 * Validate that answers are based on selected text (for selected-text queries)
 * @param response - The backend response object
 * @param selectedText - The text that was selected by the user
 * @returns Whether the response is based on the selected text
 */
export function validateSelectedTextResponse(response: ResponseObject, selectedText: string): boolean {
  if (!response || !selectedText) {
    return false;
  }

  const answer = parseAnswer(response);
  const sources = extractSources(response);

  // Check if any source content contains the selected text
  const hasSelectedTextInSources = sources.some(source =>
    source.content.toLowerCase().includes(selectedText.toLowerCase())
  );

  // Check if the answer references concepts from the selected text
  const hasReferenceToSelectedText = answer.toLowerCase().includes(selectedText.toLowerCase()) ||
    selectedText.toLowerCase().split(' ').some(word =>
      word.length > 3 && answer.toLowerCase().includes(word.toLowerCase())
    );

  return hasSelectedTextInSources || hasReferenceToSelectedText;
}

/**
 * Parse a complete response object
 * @param response - The backend response object
 * @param selectedText - Optional selected text for validation
 * @returns Parsed response with answer, sources, and metadata
 */
export function parseResponse(response: ResponseObject, selectedText: string | null = null): ParsedResponse {
  if (!response) {
    throw new Error('Response object is required');
  }

  const parsedResponse: ParsedResponse = {
    answer: parseAnswer(response),
    sources: extractSources(response),
    timingInfo: processTimingInfo(response),
    providerUsed: response.provider_used || response.providerUsed || 'unknown',
    isValidForSelectedText: selectedText ? validateSelectedTextResponse(response, selectedText) : null
  };

  return parsedResponse;
}

/**
 * Format source references for display
 * @param sources - Array of source objects
 * @returns Formatted source references
 */
export function formatSourcesForDisplay(sources: SourceObject[]): SourceForDisplay[] {
  if (!Array.isArray(sources)) {
    return [];
  }

  return sources.map((source, index) => ({
    id: source.chunkId || `source-${index}`,
    content: source.content,
    sourceDocument: source.sourceDocument,
    pageNumber: source.pageNumber,
    sectionTitle: source.sectionTitle,
    similarityScore: source.similarityScore,
    displayText: formatSourceDisplayText(source)
  }));
}

/**
 * Format a single source for display
 * @param source - Source object
 * @returns Formatted source display text
 */
function formatSourceDisplayText(source: SourceObject): string {
  const parts: string[] = [];

  if (source.sectionTitle) {
    parts.push(source.sectionTitle);
  }

  if (source.sourceDocument) {
    parts.push(source.sourceDocument);
  }

  if (source.pageNumber !== null) {
    parts.push(`Page ${source.pageNumber}`);
  }

  if (source.similarityScore) {
    parts.push(`Relevance: ${(source.similarityScore * 100).toFixed(1)}%`);
  }

  return parts.join(' - ');
}

/**
 * Get summary statistics from response
 * @param response - The parsed response object
 * @returns Summary statistics
 */
export function getResponseStats(response: ParsedResponse | ResponseObject): ResponseStats {
  if (!response) {
    return {
      sourceCount: 0,
      avgSimilarity: 0,
      totalProcessingTime: 0
    };
  }

  const sources = Array.isArray((response as ParsedResponse).sources) ?
    (response as ParsedResponse).sources : extractSources(response);
  const timingInfo = (response as ParsedResponse).timingInfo || processTimingInfo(response);

  const avgSimilarity = sources.length > 0
    ? sources.reduce((sum, source) => sum + source.similarityScore, 0) / sources.length
    : 0;

  return {
    sourceCount: sources.length,
    avgSimilarity: parseFloat(avgSimilarity.toFixed(3)),
    totalProcessingTime: timingInfo.totalTime
  };
}

export default {
  parseAnswer,
  extractSources,
  processTimingInfo,
  validateSelectedTextResponse,
  parseResponse,
  formatSourcesForDisplay,
  getResponseStats
};