import React, { useState, useRef } from 'react';
import './QueryInput.css';

/**
 * QueryInput Component
 * Provides a text area for users to input their questions
 */
const QueryInput = ({ onQuerySubmit, selectedText = null, onSelectedTextChange = null }) => {
  const [query, setQuery] = useState('');
  const [characterCount, setCharacterCount] = useState(0);
  const [useSelectedText, setUseSelectedText] = useState(false);
  const textareaRef = useRef(null);

  const MAX_QUERY_LENGTH = 10000;

  const handleSubmit = (e) => {
    e.preventDefault();

    // Sanitize the query
    const sanitizedQuery = sanitizeQuery(query);

    if (sanitizedQuery.trim() === '') {
      alert('Please enter a question');
      return;
    }

    if (onQuerySubmit) {
      // If using selected text, prepend it to the query
      let finalQuery = sanitizedQuery;
      if (useSelectedText && selectedText) {
        finalQuery = `Based on the following text: "${selectedText}", ${sanitizedQuery}`;
      }

      onQuerySubmit(finalQuery, useSelectedText ? selectedText : null);
    }

    // Clear the input after submission
    setQuery('');
    setCharacterCount(0);
  };

  const handleTextChange = (e) => {
    const text = e.target.value;

    // Limit the length
    if (text.length <= MAX_QUERY_LENGTH) {
      setQuery(text);
      setCharacterCount(text.length);
    }
  };

  const sanitizeQuery = (queryText) => {
    // Basic sanitization - remove script tags and other potentially dangerous content
    return queryText
      .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '')
      .replace(/javascript:/gi, 'javascript-disabled:');
  };

  return (
    <div className="query-input-container">
      <form onSubmit={handleSubmit} className="query-form">
        {selectedText && (
          <div className="selected-text-option">
            <label>
              <input
                type="checkbox"
                checked={useSelectedText}
                onChange={(e) => {
                  setUseSelectedText(e.target.checked);
                  if (onSelectedTextChange) {
                    onSelectedTextChange(e.target.checked ? selectedText : null);
                  }
                }}
              />
              Use selected text as context: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
            </label>
          </div>
        )}

        <div className="query-input-wrapper">
          <textarea
            ref={textareaRef}
            value={query}
            onChange={handleTextChange}
            placeholder="Ask a question about the book content..."
            className="query-textarea"
            rows="4"
            maxLength={MAX_QUERY_LENGTH}
          />
          <div className="character-counter">
            {characterCount} / {MAX_QUERY_LENGTH}
          </div>
        </div>

        <div className="query-input-actions">
          <button type="submit" className="submit-button" disabled={query.trim() === ''}>
            Ask Question
          </button>
        </div>
      </form>
    </div>
  );
};

export default QueryInput;