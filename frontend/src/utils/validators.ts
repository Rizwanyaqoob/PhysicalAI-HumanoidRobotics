/**
 * Validation Utilities for RAG Integration
 * Provides input validation functions for queries and other inputs
 */

// Define TypeScript interfaces
interface ValidationResult {
  isValid: boolean;
  error: string | null;
}

interface ValidationRequestResult {
  isValid: boolean;
  errors: string[];
}

interface SanitizationResult {
  sanitized: string;
  isValid: boolean;
  error: string | null;
}

/**
 * Validate query length and content (1-10000 chars as per spec)
 * @param query - Query to validate
 * @returns Validation result with isValid and error message
 */
export function validateQuery(query: string): ValidationResult {
  if (typeof query !== 'string') {
    return {
      isValid: false,
      error: 'Query must be a string'
    };
  }

  if (query.length < 1) {
    return {
      isValid: false,
      error: 'Query must be at least 1 character long'
    };
  }

  if (query.length > 10000) {
    return {
      isValid: false,
      error: 'Query must be no more than 10000 characters long'
    };
  }

  // Check if the query contains mostly whitespace
  if (query.trim().length < 3) {
    return {
      isValid: false,
      error: 'Query must contain meaningful content'
    };
  }

  return {
    isValid: true,
    error: null
  };
}

/**
 * Validate selected text for context-restricted queries
 * @param selectedText - Selected text to validate
 * @returns Validation result with isValid and error message
 */
export function validateSelectedText(selectedText: string): ValidationResult {
  if (typeof selectedText !== 'string') {
    return {
      isValid: false,
      error: 'Selected text must be a string'
    };
  }

  if (selectedText.length < 1) {
    return {
      isValid: false,
      error: 'Selected text cannot be empty'
    };
  }

  if (selectedText.length > 5000) {
    return {
      isValid: false,
      error: 'Selected text exceeds maximum length of 5000 characters'
    };
  }

  // Check if the selected text contains mostly whitespace
  if (selectedText.trim().replace(/\s/g, '').length < 3) {
    return {
      isValid: false,
      error: 'Selected text must contain meaningful content'
    };
  }

  return {
    isValid: true,
    error: null
  };
}

/**
 * Validate provider name
 * @param provider - Provider name to validate
 * @returns Validation result with isValid and error message
 */
export function validateProvider(provider: string): ValidationResult {
  if (typeof provider !== 'string') {
    return {
      isValid: false,
      error: 'Provider must be a string'
    };
  }

  const validProviders = ['gemini'];
  if (!validProviders.includes(provider.toLowerCase())) {
    return {
      isValid: false,
      error: `Provider must be one of: ${validProviders.join(', ')}`
    };
  }

  return {
    isValid: true,
    error: null
  };
}

/**
 * Validate max chunks parameter
 * @param maxChunks - Max chunks to validate
 * @returns Validation result with isValid and error message
 */
export function validateMaxChunks(maxChunks: number): ValidationResult {
  if (typeof maxChunks !== 'number') {
    return {
      isValid: false,
      error: 'Max chunks must be a number'
    };
  }

  if (maxChunks < 1 || maxChunks > 20) {
    return {
      isValid: false,
      error: 'Max chunks must be between 1 and 20'
    };
  }

  return {
    isValid: true,
    error: null
  };
}

/**
 * Validate user ID
 * @param userId - User ID to validate
 * @returns Validation result with isValid and error message
 */
export function validateUserId(userId: string | null | undefined): ValidationResult {
  // User ID is optional, so it can be null/undefined
  if (userId === null || userId === undefined) {
    return {
      isValid: true,
      error: null
    };
  }

  if (typeof userId !== 'string') {
    return {
      isValid: false,
      error: 'User ID must be a string'
    };
  }

  if (userId.length < 1 || userId.length > 100) {
    return {
      isValid: false,
      error: 'User ID must be between 1 and 100 characters long'
    };
  }

  return {
    isValid: true,
    error: null
  };
}

/**
 * Validate a complete query request
 * @param query - Query string
 * @param provider - Provider name
 * @param maxChunks - Max chunks
 * @param selectedText - Optional selected text
 * @param userId - Optional user ID
 * @returns Validation result with isValid and array of errors
 */
export function validateQueryRequest(
  query: string,
  provider: string,
  maxChunks: number,
  selectedText: string | null = null,
  userId: string | null = null
): ValidationRequestResult {
  const errors: string[] = [];

  // Validate query
  const queryValidation = validateQuery(query);
  if (!queryValidation.isValid) {
    errors.push(`Query validation error: ${queryValidation.error}`);
  }

  // Validate provider
  const providerValidation = validateProvider(provider);
  if (!providerValidation.isValid) {
    errors.push(`Provider validation error: ${providerValidation.error}`);
  }

  // Validate max chunks
  const maxChunksValidation = validateMaxChunks(maxChunks);
  if (!maxChunksValidation.isValid) {
    errors.push(`Max chunks validation error: ${maxChunksValidation.error}`);
  }

  // Validate selected text if provided
  if (selectedText) {
    const selectedTextValidation = validateSelectedText(selectedText);
    if (!selectedTextValidation.isValid) {
      errors.push(`Selected text validation error: ${selectedTextValidation.error}`);
    }
  }

  // Validate user ID if provided
  if (userId) {
    const userIdValidation = validateUserId(userId);
    if (!userIdValidation.isValid) {
      errors.push(`User ID validation error: ${userIdValidation.error}`);
    }
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Sanitize and validate query
 * @param query - Query to sanitize and validate
 * @returns Result with sanitized query and validation status
 */
export function sanitizeAndValidateQuery(query: string): SanitizationResult {
  // First sanitize the query
  const sanitizedQuery = query
    .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '')
    .replace(/<iframe\b[^<]*(?:(?!<\/iframe>)<[^<]*)*<\/iframe>/gi, '')
    .replace(/javascript:/gi, '')
    .replace(/on\w+\s*=/gi, '');

  // Then validate
  const validation = validateQuery(sanitizedQuery);

  return {
    sanitized: sanitizedQuery,
    isValid: validation.isValid,
    error: validation.error
  };
}

/**
 * Sanitize and validate selected text
 * @param selectedText - Selected text to sanitize and validate
 * @returns Result with sanitized text and validation status
 */
export function sanitizeAndValidateSelectedText(selectedText: string): SanitizationResult {
  // First sanitize the selected text
  const sanitizedText = selectedText
    .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '')
    .replace(/<iframe\b[^<]*(?:(?!<\/iframe>)<[^<]*)*<\/iframe>/gi, '')
    .replace(/javascript:/gi, '')
    .replace(/on\w+\s*=/gi, '');

  // Then validate
  const validation = validateSelectedText(sanitizedText);

  return {
    sanitized: sanitizedText,
    isValid: validation.isValid,
    error: validation.error
  };
}

// Export all validator functions
export default {
  validateQuery,
  validateSelectedText,
  validateProvider,
  validateMaxChunks,
  validateUserId,
  validateQueryRequest,
  sanitizeAndValidateQuery,
  sanitizeAndValidateSelectedText
};