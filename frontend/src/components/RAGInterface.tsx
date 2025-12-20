import React, { useState, useEffect } from 'react';
import QueryInput from './QueryInput';
import ResponseDisplay from './ResponseDisplay';
import LoadingIndicator from './LoadingIndicator';
import ErrorDisplay from './ErrorDisplay';
import { sendQuery, sendSelectedTextQuery } from '../services/query-handler';
import './RAGInterface.css';

// Define TypeScript interfaces
interface ParsedResponse {
  answer: string;
  sources: Array<any>;
  timingInfo: any;
  providerUsed: string;
  isValidForSelectedText: boolean | null;
  [key: string]: any;
}

/**
 * Main RAG Interface Component
 * Integrates all components into a cohesive interface for Docusaurus
 */
const RAGInterface: React.FC = () => {
  const [query, setQuery] = useState<string>('');
  const [response, setResponse] = useState<ParsedResponse | null>(null);
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [useSelectedText, setUseSelectedText] = useState<boolean>(false);

  // Capture selected text from the page
  const captureSelectedText = (): string => {
    const selectedText = window.getSelection()?.toString().trim() || '';
    if (selectedText) {
      setSelectedText(selectedText);
    }
    return selectedText;
  };

  // Handle query submission
  const handleQuerySubmit = async (queryText: string, selectedTextContext: string | null = null) => {
    setError(null);
    setLoading(true);

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
    } catch (err: any) {
      setError(err.message || 'An error occurred while processing your query');
    } finally {
      setLoading(false);
    }
  };

  // Handle retry
  const handleRetry = () => {
    if (query) {
      handleQuerySubmit(query, useSelectedText ? selectedText : null);
    }
  };

  // Setup text selection listener when component mounts
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

  return (
    <div className="rag-interface-container">
      <div className="rag-interface-header">
        <h2>Ask Questions About the Book</h2>
      </div>

      <QueryInput
        onQuerySubmit={handleQuerySubmit}
        selectedText={selectedText}
        onSelectedTextChange={(text) => setUseSelectedText(!!text)}
      />

      {loading && (
        <LoadingIndicator
          show={true}
          message="Processing your question..."
          estimatedTime={2}
        />
      )}

      {error && (
        <ErrorDisplay
          error={error}
          showError={true}
          onRetry={handleRetry}
          showRetryButton={true}
        />
      )}

      {response && !loading && (
        <ResponseDisplay
          response={response}
          show={true}
        />
      )}
    </div>
  );
};

export default RAGInterface;