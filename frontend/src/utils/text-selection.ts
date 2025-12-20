/**
 * Text Selection Utilities for RAG Integration
 * Handles capturing and processing selected text from the page
 */

// Define TypeScript interfaces
interface SelectionInfo {
  text: string;
  startOffset: number | null;
  endOffset: number | null;
  startContainer: Node | null;
  endContainer: Node | null;
  range: Range | null;
}

interface ValidationResult {
  isValid: boolean;
  errors: string[];
}

/**
 * Get the currently selected text from the page
 * @returns The selected text, or empty string if no text is selected
 */
export function getSelectedText(): string {
  return window.getSelection ? window.getSelection()!.toString().trim() : '';
}

/**
 * Get detailed information about the current text selection
 * @returns Selection information including text, position, and context
 */
export function getSelectionInfo(): SelectionInfo {
  const selection = window.getSelection ? window.getSelection() : null;

  if (!selection || selection.toString().trim() === '') {
    return {
      text: '',
      startOffset: null,
      endOffset: null,
      startContainer: null,
      endContainer: null,
      range: null
    };
  }

  const range = selection.rangeCount > 0 ? selection.getRangeAt(0) : null;

  return {
    text: selection.toString().trim(),
    startOffset: range ? range.startOffset : null,
    endOffset: range ? range.endOffset : null,
    startContainer: range ? range.startContainer : null,
    endContainer: range ? range.endContainer : null,
    range: range
  };
}

/**
 * Sanitize selected text to remove potentially dangerous content
 * @param text - The selected text to sanitize
 * @returns Sanitized text
 */
export function sanitizeSelectedText(text: string): string {
  if (typeof text !== 'string') {
    return '';
  }

  // Remove any HTML tags that might be in the selection
  const sanitized = text.replace(/<[^>]*>/g, '');

  // Limit length to prevent abuse
  if (sanitized.length > 5000) {
    return sanitized.substring(0, 5000);
  }

  return sanitized;
}

/**
 * Validate selected text for use in context-restricted queries
 * @param text - The selected text to validate
 * @returns Validation result with isValid and errors
 */
export function validateSelectedText(text: string): ValidationResult {
  const errors: string[] = [];

  if (!text || typeof text !== 'string') {
    errors.push('Selected text is required and must be a string');
  } else {
    if (text.trim().length === 0) {
      errors.push('Selected text cannot be empty');
    }

    if (text.length > 5000) {
      errors.push('Selected text exceeds maximum length of 5,000 characters');
    }

    if (text.trim().split(/\s+/).length < 2) {
      errors.push('Selected text should contain at least 2 words for meaningful context');
    }
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}

/**
 * Get the parent element of the current selection
 * @returns The parent element of the selection
 */
export function getSelectionParentElement(): Element | null {
  const selection = window.getSelection ? window.getSelection() : null;

  if (!selection || selection.rangeCount === 0) {
    return null;
  }

  const range = selection.getRangeAt(0);
  return range.commonAncestorContainer.nodeType === 1
    ? range.commonAncestorContainer as Element
    : range.commonAncestorContainer.parentElement;
}

/**
 * Highlight selected text with a temporary visual indicator
 * @param color - The highlight color (default: yellow)
 */
export function highlightSelection(color: string = '#ffff99'): void {
  const selection = window.getSelection ? window.getSelection() : null;

  if (!selection || selection.toString().trim() === '') {
    return;
  }

  const range = selection.getRangeAt(0);
  const span = document.createElement('span');
  span.style.backgroundColor = color;
  span.style.transition = 'background-color 0.5s ease';

  range.surroundContents(span);

  // Remove highlight after a delay
  setTimeout(() => {
    const parent = span.parentElement;
    if (parent) {
      parent.replaceChild(document.createTextNode(span.textContent!), span);
    }
  }, 2000);
}

/**
 * Add event listener for text selection
 * @param callback - Function to call when text is selected
 * @returns Function to remove the event listener
 */
export function addSelectionListener(callback: (selectionInfo: SelectionInfo) => void): () => void {
  const handleSelectionChange = () => {
    const selectionInfo = getSelectionInfo();
    if (selectionInfo.text) {
      callback(selectionInfo);
    }
  };

  document.addEventListener('selectionchange', handleSelectionChange);

  // Return function to remove the listener
  return () => {
    document.removeEventListener('selectionchange', handleSelectionChange);
  };
}

/**
 * Format selected text for use in queries
 * @param text - The selected text to format
 * @returns Formatted text ready for queries
 */
export function formatSelectedTextForQuery(text: string): string {
  if (!text) return '';

  // Sanitize the text
  const sanitized = sanitizeSelectedText(text);

  // Remove extra whitespace and normalize
  const normalized = sanitized
    .replace(/\s+/g, ' ')
    .trim();

  return normalized;
}

export default {
  getSelectedText,
  getSelectionInfo,
  sanitizeSelectedText,
  validateSelectedText,
  getSelectionParentElement,
  highlightSelection,
  addSelectionListener,
  formatSelectedTextForQuery
};