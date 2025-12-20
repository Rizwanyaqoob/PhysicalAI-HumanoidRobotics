import React, { useState } from 'react';
import { sanitizeHTML, sanitizeText } from '../utils/sanitizer';
import './ResponseDisplay.css';

/**
 * ResponseDisplay Component
 * Displays the answer from the RAG agent along with source references
 */
const ResponseDisplay = ({ response, show = false }) => {
  const [expandedSources, setExpandedSources] = useState({});
  const [copiedSourceId, setCopiedSourceId] = useState(null);

  if (!show || !response) {
    return null;
  }

  const handleSourceToggle = (index) => {
    setExpandedSources(prev => ({
      ...prev,
      [index]: !prev[index]
    }));
  };

  const handleCopyAnswer = () => {
    if (response.answer) {
      navigator.clipboard.writeText(response.answer);
    }
  };

  const handleCopySource = (sourceId, content) => {
    navigator.clipboard.writeText(content);
    setCopiedSourceId(sourceId);
    setTimeout(() => setCopiedSourceId(null), 2000); // Reset after 2 seconds
  };

  // Sanitize the answer content
  const sanitizedAnswer = sanitizeHTML(response.answer || '');

  return (
    <div className="response-display-container">
      <div className="response-header">
        <h3>Answer</h3>
        <button className="copy-answer-button" onClick={handleCopyAnswer} title="Copy answer">
          Copy Answer
        </button>
      </div>

      <div className="response-answer">
        {sanitizedAnswer ? (
          <div
            className="answer-content"
            dangerouslySetInnerHTML={{ __html: sanitizedAnswer }}
          />
        ) : (
          <p className="no-answer">No answer provided.</p>
        )}
      </div>

      {response.sources && response.sources.length > 0 && (
        <div className="response-sources">
          <h4>Sources:</h4>
          <div className="sources-list">
            {response.sources.map((source, index) => (
              <div key={source.chunk_id || `source-${index}`} className="source-item">
                <div className="source-header">
                  <div className="source-info">
                    <span className="source-title">
                      {sanitizeText(source.section_title) || 'Untitled Section'}
                    </span>
                    {source.page_number && (
                      <span className="source-page">Page {source.page_number}</span>
                    )}
                    <span className="source-document">
                      {sanitizeText(source.source_document)}
                    </span>
                  </div>
                  <div className="source-actions">
                    <button
                      className="copy-source-button"
                      onClick={() => handleCopySource(source.chunk_id || index, source.content)}
                      title="Copy source content"
                    >
                      {copiedSourceId === (source.chunk_id || index) ? 'Copied!' : 'Copy'}
                    </button>
                    <button
                      className="toggle-source-button"
                      onClick={() => handleSourceToggle(index)}
                      title={expandedSources[index] ? 'Collapse' : 'Expand'}
                    >
                      {expandedSources[index] ? '▲' : '▼'}
                    </button>
                  </div>
                </div>

                <div
                  className={`source-content ${expandedSources[index] ? 'expanded' : 'collapsed'}`}
                >
                  <div className="source-similarity">
                    Relevance: {(source.similarity_score * 100).toFixed(1)}%
                  </div>
                  <div
                    className="source-text"
                    dangerouslySetInnerHTML={{
                      __html: sanitizeHTML(source.content)
                    }}
                  />
                </div>
              </div>
            ))}
          </div>
        </div>
      )}

      {response.timingInfo && (
        <div className="response-timing">
          <div className="timing-info">
            <span>Embedding: {response.timingInfo.embeddingTime.toFixed(3)}s</span>
            <span>Retrieval: {response.timingInfo.retrievalTime.toFixed(3)}s</span>
            <span>Generation: {response.timingInfo.generationTime.toFixed(3)}s</span>
            <span className="total-time">Total: {response.timingInfo.totalTime.toFixed(3)}s</span>
          </div>
        </div>
      )}
    </div>
  );
};

export default ResponseDisplay;