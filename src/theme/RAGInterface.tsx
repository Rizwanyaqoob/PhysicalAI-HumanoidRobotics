import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Import the RAG components from the frontend directory
// Since these are in a separate directory, we'll need to make them available to Docusaurus
// For now, I'll create a simplified version that demonstrates the integration

interface RAGResponse {
  answer: string;
  sources: Array<any>;
  timingInfo: any;
  providerUsed: string;
}

const RAGInterface: React.FC = () => {
  const [query, setQuery] = useState<string>('');
  const [response, setResponse] = useState<RAGResponse | null>(null);
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const [useSelectedText, setUseSelectedText] = useState<boolean>(false);

  const { siteConfig } = useDocusaurusContext();
  const backendUrl = process.env.BACKEND_API_URL || siteConfig.customFields?.BACKEND_API_URL || 'http://localhost:8009';

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
  const handleQuerySubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      const requestBody = {
        query: useSelectedText && selectedText ? `Based on the following text: "${selectedText}", ${query}` : query,
        provider: 'gemini',
        max_chunks: 5
      };

      const response = await fetch(`${backendUrl}/api/ask`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`Backend error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();
      setResponse(data);
    } catch (err: any) {
      setError(err.message || 'An error occurred while processing your query');
    } finally {
      setLoading(false);
    }
  };

  // Handle retry
  const handleRetry = () => {
    if (query) {
      const formEvent = { preventDefault: () => {} } as React.FormEvent;
      handleQuerySubmit(formEvent);
    }
  };

  return (
    <div className="rag-interface-container" style={{
      border: '1px solid #ddd',
      borderRadius: '8px',
      padding: '16px',
      margin: '16px 0',
      backgroundColor: '#f9f9f9'
    }}>
      <h3 style={{ margin: '0 0 16px 0', color: '#2c3e50' }}>Ask Questions About the Book</h3>

      <form onSubmit={handleQuerySubmit}>
        <div style={{ marginBottom: '12px' }}>
          <textarea
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            placeholder="Ask a question about the book content..."
            style={{
              width: '100%',
              minHeight: '80px',
              padding: '8px',
              border: '1px solid #ccc',
              borderRadius: '4px',
              resize: 'vertical'
            }}
            disabled={loading}
          />
        </div>

        {selectedText && (
          <div style={{ marginBottom: '12px' }}>
            <label style={{ display: 'flex', alignItems: 'center' }}>
              <input
                type="checkbox"
                checked={useSelectedText}
                onChange={(e) => setUseSelectedText(e.target.checked)}
                style={{ marginRight: '8px' }}
              />
              Restrict to selected text: <em>"{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"</em>
            </label>
          </div>
        )}

        <button
          type="submit"
          disabled={loading || !query.trim()}
          style={{
            backgroundColor: loading ? '#ccc' : '#3578e5',
            color: 'white',
            border: 'none',
            padding: '8px 16px',
            borderRadius: '4px',
            cursor: (loading || !query.trim()) ? 'not-allowed' : 'pointer'
          }}
        >
          {loading ? 'Processing...' : 'Ask Question'}
        </button>
      </form>

      {loading && (
        <div style={{
          padding: '16px',
          textAlign: 'center',
          color: '#666'
        }}>
          <div>Processing your question...</div>
          <div style={{ marginTop: '8px' }}>
            <div style={{
              display: 'inline-block',
              width: '20px',
              height: '20px',
              border: '3px solid #f3f3f3',
              borderTop: '3px solid #3578e5',
              borderRadius: '50%',
              animation: 'spin 1s linear infinite'
            }}></div>
          </div>
        </div>
      )}

      {error && (
        <div style={{
          padding: '12px',
          backgroundColor: '#ffebee',
          border: '1px solid #ffcdd2',
          borderRadius: '4px',
          color: '#c62828',
          margin: '12px 0'
        }}>
          <div><strong>Error:</strong> {error}</div>
          <button
            onClick={handleRetry}
            style={{
              marginTop: '8px',
              padding: '4px 8px',
              backgroundColor: '#3578e5',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer'
            }}
          >
            Retry
          </button>
        </div>
      )}

      {response && !loading && (
        <div style={{
          marginTop: '16px',
          padding: '16px',
          backgroundColor: 'white',
          border: '1px solid #e0e0e0',
          borderRadius: '4px'
        }}>
          <h4 style={{ margin: '0 0 12px 0', color: '#2c3e50' }}>Answer</h4>
          <div style={{ whiteSpace: 'pre-wrap', lineHeight: '1.6' }}>
            {response.answer}
          </div>

          {response.sources && response.sources.length > 0 && (
            <div style={{ marginTop: '16px' }}>
              <h5 style={{ margin: '0 0 8px 0', color: '#555' }}>Sources:</h5>
              <ul style={{ margin: '8px 0', paddingLeft: '20px' }}>
                {response.sources.map((source: any, index: number) => (
                  <li key={index} style={{ marginBottom: '8px' }}>
                    <strong>{source.source_document}</strong>
                    {source.page_number && <span>, Page: {source.page_number}</span>}
                    {source.section_title && <span>, Section: {source.section_title}</span>}
                    <div style={{ fontSize: '0.9em', color: '#666', marginTop: '4px' }}>
                      {source.content.substring(0, 200)}{source.content.length > 200 ? '...' : ''}
                    </div>
                  </li>
                ))}
              </ul>
            </div>
          )}

          {response.debug_info && (
            <div style={{
              marginTop: '12px',
              fontSize: '0.8em',
              color: '#777',
              borderTop: '1px solid #eee',
              paddingTop: '8px'
            }}>
              Provider: {response.providerUsed} | Total time: {response.debug_info.total_time?.toFixed(2)}s
            </div>
          )}
        </div>
      )}

      <style jsx>{`
        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }
      `}</style>
    </div>
  );
};

// BrowserOnly wrapper to ensure this only runs in the browser
const RAGInterfaceWrapper: React.FC = () => {
  return (
    <BrowserOnly>
      {() => <RAGInterface />}
    </BrowserOnly>
  );
};

export default RAGInterfaceWrapper;