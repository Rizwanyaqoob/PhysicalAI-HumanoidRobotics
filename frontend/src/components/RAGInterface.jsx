import React, { useState } from 'react';
import QueryInput from './QueryInput';
import ResponseDisplay from './ResponseDisplay';
import LoadingIndicator from './LoadingIndicator';
import ErrorDisplay from './ErrorDisplay';
import { sendQuery, sendSelectedTextQuery } from '../services/query-handler';
import './RAGInterface.css';

/**
 * Main RAG Interface Component
 * Integrates all components into a cohesive interface for Docusaurus
 */
const RAGInterface = () => {
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState(null);
  const [useSelectedText, setUseSelectedText] = useState(false);

  // Capture selected text from the page
  const captureSelectedText = () => {
    const selectedText = window.getSelection().toString().trim();
    if (selectedText) {
      setSelectedText(selectedText);
    }
    return selectedText;
  };

  // Handle query submission
  const handleQuerySubmit = async (queryText, selectedTextContext = null) => {
    setError(null);
    setLoading(true);

    try {
      let result;
      if (selectedTextContext) {
        // Use selected text query
        result = await sendSelectedTextQuery(queryText, selectedTextContext);
      } else {
        // Use full-book query
        result = await sendQuery(queryText);
      }

      setResponse(result);
    } catch (err) {
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
  React.useEffect(() => {
    const handleSelectionChange = () => {
      const text = window.getSelection().toString().trim();
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