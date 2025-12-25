import React, { useState, useEffect, useCallback } from 'react';
import { ChatInput } from './ChatInput';
import { ChatAnswer } from './ChatAnswer';
import { ChatSources } from './ChatSources';
import { ChatStatus } from './ChatStatus';
import { sendQuery, sendSelectedTextQuery } from '../services/query-handler';
import { ParsedResponse } from '../types/rag-types';
import './RAGChat.css';

// Define the possible states for the chat
type ChatState = 'idle' | 'searching' | 'answering' | 'sources' | 'error';

interface RAGChatProps {
  variant?: 'floating' | 'sidebar' | 'inline'; // Different UI variants
  className?: string; // Additional CSS classes
  placeholder?: string; // Input placeholder text
  title?: string; // Chat title
}

/**
 * Core RAGChat Component
 * Handles all the logic for RAG-based chat functionality
 */
export const RAGChat: React.FC<RAGChatProps> = ({
  variant = 'sidebar',
  className = '',
  placeholder = 'Ask a question about the book content...',
  title = 'Ask Questions About the Book'
}) => {
  const [query, setQuery] = useState<string>('');
  const [response, setResponse] = useState<ParsedResponse | null>(null);
  const [chatState, setChatState] = useState<ChatState>('idle');
  const [error, setError] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [useSelectedText, setUseSelectedText] = useState<boolean>(false);

  // Capture selected text from the page
  useEffect(() => {
    const handleSelectionChange = () => {
      const text = window.getSelection()?.toString().trim() || '';
      if (text) {
        setSelectedText(text);
      }
    };

    document.addEventListener('selectionchange', handleSelectionChange);
    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, []);

  // Handle query submission
  const handleQuerySubmit = useCallback(async (queryText: string, selectedTextContext: string | null = null) => {
    setError(null);
    setChatState('searching');

    try {
      let result: ParsedResponse;
      if (selectedTextContext) {
        // Use selected text query
        result = await sendSelectedTextQuery(queryText, selectedTextContext);
      } else {
        // Use full-book query
        result = await sendQuery(queryText);
      }

      setResponse(result);

      // Transition to answering state first, then to sources if there are sources
      setChatState('answering');

      // If there are sources, transition to sources state after a brief delay
      // to show the answer first
      if (result.sources && result.sources.length > 0) {
        setTimeout(() => {
          setChatState('sources');
        }, 500); // Small delay to show answer before sources
      }
    } catch (err: any) {
      setError(err.message || 'An error occurred while processing your query');
      setChatState('error');
    }
  }, []);

  // Handle retry
  const handleRetry = useCallback(() => {
    if (query) {
      setChatState('searching');
      handleQuerySubmit(query, useSelectedText ? selectedText : null);
    }
  }, [query, useSelectedText, selectedText, handleQuerySubmit]);

  // Handle query submission from input component
  const onSubmit = useCallback((queryText: string) => {
    setQuery(queryText);
    handleQuerySubmit(queryText, useSelectedText ? selectedText : null);
  }, [handleQuerySubmit, useSelectedText, selectedText]);

  // Handle selected text toggle
  const onSelectedTextToggle = useCallback((useText: boolean) => {
    setUseSelectedText(useText);
  }, []);

  return (
    <div className={`rag-chat-container rag-chat-${variant} ${className}`}>
      <div className="rag-chat-header">
        <h2>{title}</h2>
      </div>

      <ChatInput
        onSubmit={onSubmit}
        selectedText={selectedText}
        onSelectedTextToggle={onSelectedTextToggle}
        useSelectedText={useSelectedText}
        placeholder={placeholder}
        disabled={chatState === 'searching'}
      />

      {chatState === 'searching' && (
        <ChatStatus
          status="loading"
          message="Processing your question..."
          estimatedTime={2}
        />
      )}

      {chatState === 'error' && error && (
        <ChatStatus
          status="error"
          message={error}
          onRetry={handleRetry}
          showRetryButton={true}
        />
      )}

      {response && (chatState === 'answering' || chatState === 'sources') && (
        <ChatAnswer answer={response.answer} />
      )}

      {response && response.sources && response.sources.length > 0 && chatState === 'sources' && (
        <ChatSources sources={response.sources} />
      )}
    </div>
  );
};

export default RAGChat;