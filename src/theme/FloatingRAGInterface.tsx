import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Floating RAG Interface Component
const FloatingRAGInterface = () => {
  const { siteConfig } = useDocusaurusContext();
  const [isOpen, setIsOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);
  const [selectedText, setSelectedText] = useState(null);
  const [useSelectedText, setUseSelectedText] = useState(false);

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

  const handleQuerySubmit = async (e) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    // First, try the actual backend API
    try {
      const backendUrl = siteConfig.customFields?.BACKEND_API_URL || 'https://rizwanya-chatbotdeploy.hf.space';
      console.log('Backend URL:', backendUrl);
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
    } catch (err) {
      console.error('Backend API error:', err);

      // Fallback to mock response for UI testing
      console.log('Using mock response for UI testing');
      setResponse({
        answer: "This is a mock response for UI testing purposes. The actual backend is currently unavailable or has connectivity issues. When the backend is working, you would see real answers to your questions about humanoid robotics here.",
        sources: [
          {
            content: "This is sample content from the robotics documentation that would normally be retrieved from the vector database.",
            source_document: "sample_document.md",
            page_number: 42,
            section_title: "Introduction to Humanoid Robotics",
            similarity_score: 0.85,
            chunk_id: "mock-chunk-1"
          }
        ],
        providerUsed: "mock-provider",
        debug_info: {
          embedding_time: 0.12,
          retrieval_time: 0.34,
          generation_time: 0.56,
          total_time: 1.02
        }
      });
    } finally {
      setLoading(false);
    }
  };

  const handleRetry = () => {
    if (query) {
      const formEvent = { preventDefault: () => {} };
      handleQuerySubmit(formEvent);
    }
  };

  return (
    <div className="floating-rag-container">
      <button
        className="rag-toggle-button"
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? "Close RAG interface" : "Open RAG interface"}
      >
        {isOpen ? '✕' : '?'}
      </button>

      {isOpen && (
        <div className="rag-interface-modal">
          <div className="rag-interface-content">
            <h3 style={{ margin: '0 0 16px 0', color: '#2c3e50' }}>Ask Questions About the Book</h3>

            <form onSubmit={handleQuerySubmit} style={{ marginBottom: '16px' }}>
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
                      {response.sources.map((source, index) => (
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
          </div>
        </div>
      )}

      <style jsx>{`
        .floating-rag-container {
          position: fixed;
          bottom: 20px;
          right: 20px;
          z-index: 9999 !important; /* Highest possible z-index */
        }

        .rag-toggle-button {
          width: 60px;
          height: 60px;
          border-radius: 50%;
          border: 2px solid white;
          background-color: #3578e5;
          color: white;
          font-size: 24px;
          font-weight: bold;
          cursor: pointer;
          box-shadow: 0 4px 12px rgba(0,0,0,0.4);
          display: flex;
          align-items: center;
          justify-content: center;
          transition: all 0.2s ease;
        }

        .rag-toggle-button:hover {
          transform: scale(1.1);
          box-shadow: 0 6px 16px rgba(0,0,0,0.5);
          background-color: #2a64c5;
        }

        .rag-interface-modal {
          position: absolute;
          bottom: 60px;
          right: 0;
          width: 400px;
          max-width: calc(100vw - 40px);
          background: white;
          border-radius: 8px;
          box-shadow: 0 4px 20px rgba(0,0,0,0.3);
          overflow: hidden;
        }

        .rag-interface-content {
          padding: 20px;
          max-height: 70vh;
          overflow-y: auto;
        }

        @keyframes spin {
          0% { transform: rotate(0deg); }
          100% { transform: rotate(360deg); }
        }

        @media (max-width: 480px) {
          .rag-interface-modal {
            width: calc(100vw - 20px);
            left: 10px;
            right: 10px;
          }
        }
      `}</style>
    </div>
  );
};

const FloatingRAGInterfaceWrapper = () => {
  return (
    <BrowserOnly>
      {() => <FloatingRAGInterface />}
    </BrowserOnly>
  );
};

export default FloatingRAGInterfaceWrapper;