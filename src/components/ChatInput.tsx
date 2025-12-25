import React, { useState, useRef, KeyboardEvent } from 'react';
import './ChatInput.css';

interface ChatInputProps {
  onSubmit: (input: string) => void;
  isLoading: boolean;
  selectedText?: string | null;
}

export const ChatInput: React.FC<ChatInputProps> = ({ onSubmit, isLoading, selectedText }) => {
  const [input, setInput] = useState('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  const handleSubmit = () => {
    if (input.trim() && !isLoading) {
      onSubmit(input);
      setInput('');
      // Reset textarea height
      if (textareaRef.current) {
        textareaRef.current.style.height = 'auto';
      }
    }
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  const handleInput = () => {
    if (textareaRef.current) {
      // Auto-resize textarea
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = `${Math.min(textareaRef.current.scrollHeight, 150)}px`;
    }
  };

  return (
    <div className="chat-input-container">
      {selectedText && (
        <div className="selected-text-preview">
          <span className="selected-text-label">Selected text:</span>
          <span className="selected-text-content">
            "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
          </span>
        </div>
      )}
      <div className="chat-input-wrapper">
        <textarea
          ref={textareaRef}
          value={input}
          onChange={(e) => {
            setInput(e.target.value);
            handleInput();
          }}
          onKeyDown={handleKeyDown}
          placeholder="Message..."
          className="chat-input-textarea"
          rows={1}
          disabled={isLoading}
        />
        <button
          onClick={handleSubmit}
          disabled={!input.trim() || isLoading}
          className="chat-submit-button"
        >
          {isLoading ? 'Sending...' : 'Send'}
        </button>
      </div>
    </div>
  );
};