import React, { useState, useRef } from 'react';
import './ChatInput.css';

interface ChatInputProps {
  onSubmit: (query: string) => void;
  selectedText?: string | null;
  onSelectedTextToggle?: (useSelectedText: boolean) => void;
  useSelectedText?: boolean;
  placeholder?: string;
  disabled?: boolean;
}

/**
 * ChatInput Component
 * Provides a text area for users to input their questions
 */
export const ChatInput: React.FC<ChatInputProps> = ({
  onSubmit,
  selectedText = null,
  onSelectedTextToggle = () => {},
  useSelectedText = false,
  placeholder = 'Ask a question...',
  disabled = false
}) => {
  const [query, setQuery] = useState('');
  const [characterCount, setCharacterCount] = useState(0);
  const [localUseSelectedText, setLocalUseSelectedText] = useState(useSelectedText);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  const MAX_QUERY_LENGTH = 10000;

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();

    // Sanitize the query
    const sanitizedQuery = sanitizeQuery(query);

    if (sanitizedQuery.trim() === '') {
      alert('Please enter a question');
      return;
    }

    onSubmit(sanitizedQuery);

    // Clear the input after submission
    setQuery('');
    setCharacterCount(0);
  };

  const handleTextChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const text = e.target.value;

    // Limit the length
    if (text.length <= MAX_QUERY_LENGTH) {
      setQuery(text);
      setCharacterCount(text.length);
    }
  };

  const handleSelectedTextToggle = (e: React.ChangeEvent<HTMLInputElement>) => {
    const checked = e.target.checked;
    setLocalUseSelectedText(checked);
    onSelectedTextToggle(checked);
  };

  // Auto-clear selected text if user starts typing in the query box
  const handleQueryChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const text = e.target.value;

    // If user starts typing while selected text is active,
    // we'll clear the selected text usage to avoid confusion
    if (text.length > 0 && localUseSelectedText && selectedText) {
      setLocalUseSelectedText(false);
      onSelectedTextToggle(false);
    }

    // Limit the length
    if (text.length <= MAX_QUERY_LENGTH) {
      setQuery(text);
      setCharacterCount(text.length);
    }
  };

  const sanitizeQuery = (queryText: string): string => {
    // Basic sanitization - remove script tags and other potentially dangerous content
    return queryText
      .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '')
      .replace(/javascript:/gi, 'javascript-disabled:');
  };

  return (
    <div className="chat-input-container">
      <form onSubmit={handleSubmit} className="chat-form">
        {selectedText && (
          <div className="selected-text-option">
            <label className="selected-text-label">
              <input
                type="checkbox"
                checked={localUseSelectedText}
                onChange={handleSelectedTextToggle}
                className="selected-text-checkbox"
              />
              <div className="selected-text-preview">
                <span className="selected-text-label-text">
                  Use selected text as context:
                </span>
                <div className="selected-text-content">
                  <em>"{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</em>
                </div>
              </div>
            </label>
          </div>
        )}

        <div className="chat-input-wrapper">
          <textarea
            ref={textareaRef}
            value={query}
            onChange={handleQueryChange}
            placeholder={placeholder}
            className="chat-textarea"
            rows={4}
            maxLength={MAX_QUERY_LENGTH}
            disabled={disabled}
          />
          <div className="character-counter">
            {characterCount} / {MAX_QUERY_LENGTH}
          </div>
        </div>

        <div className="chat-input-actions">
          <button
            type="submit"
            className="submit-button"
            disabled={disabled || query.trim() === ''}
          >
            Ask Question
          </button>
        </div>
      </form>
    </div>
  );
};