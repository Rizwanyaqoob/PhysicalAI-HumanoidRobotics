import React, { useState } from 'react';
import { Source } from '../frontend/src/types/rag-types';
import { sanitizeHTML, sanitizeText } from '../frontend/src/utils/sanitizer';
import './ChatSources.css';

interface ChatSourcesProps {
  sources: Source[];
}

/**
 * ChatSources Component
 * Displays the source references from the RAG agent
 */
export const ChatSources: React.FC<ChatSourcesProps> = ({ sources }) => {
  const [expandedSources, setExpandedSources] = useState<Record<number, boolean>>({});
  const [copiedSourceId, setCopiedSourceId] = useState<string | number | null>(null);

  const handleSourceToggle = (index: number) => {
    setExpandedSources(prev => ({
      ...prev,
      [index]: !prev[index]
    }));
  };

  const handleCopySource = (sourceId: string | number, content: string) => {
    navigator.clipboard.writeText(content);
    setCopiedSourceId(sourceId);
    setTimeout(() => setCopiedSourceId(null), 2000); // Reset after 2 seconds
  };

  if (!sources || sources.length === 0) {
    return null;
  }

  return (
    <div className="chat-sources-container">
      <div className="chat-sources-header">
        <h4>Sources:</h4>
      </div>

      <div className="sources-list">
        {sources.map((source, index) => (
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
  );
};