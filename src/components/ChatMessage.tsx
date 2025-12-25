import React, { useState } from 'react';
import { sanitizeHTML } from '../frontend/src/utils/sanitizer';
import './ChatMessage.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    content: string;
    source_document: string;
    page_number?: number;
    section_title?: string;
    similarity_score: number;
    chunk_id: string;
  }>;
  timestamp: Date;
}

interface ChatMessageProps {
  message: Message;
  isStreaming?: boolean;
}

export const ChatMessage: React.FC<ChatMessageProps> = ({ message, isStreaming = false }) => {
  const [showSources, setShowSources] = useState(false);

  const isUser = message.role === 'user';
  const sanitizedContent = sanitizeHTML(message.content);

  const hasSources = message.sources && message.sources.length > 0;

  return (
    <div className={`chat-message ${isUser ? 'user-message' : 'assistant-message'}`}>
      <div className="message-content">
        {isUser ? (
          <div className="user-bubble">
            {message.content}
          </div>
        ) : (
          <div className="assistant-bubble">
            {isStreaming ? (
              <div className="typing-indicator">
                <span></span>
                <span></span>
                <span></span>
              </div>
            ) : (
              <>
                <div
                  className="message-text"
                  dangerouslySetInnerHTML={{ __html: sanitizedContent }}
                />

                {hasSources && (
                  <div className="sources-section">
                    <button
                      className="sources-toggle"
                      onClick={() => setShowSources(!showSources)}
                    >
                      Sources ({message.sources?.length || 0})
                    </button>

                    {showSources && message.sources && (
                      <div className="sources-list">
                        {message.sources.map((source, index) => (
                          <div key={source.chunk_id || `source-${index}`} className="source-item">
                            <div className="source-header">
                              <span className="source-document">
                                {source.source_document}
                              </span>
                              {source.page_number && (
                                <span className="source-page">Page {source.page_number}</span>
                              )}
                              {source.section_title && (
                                <span className="source-section">Section: {source.section_title}</span>
                              )}
                            </div>
                            <div className="source-content">
                              {source.content.substring(0, 200)}...
                            </div>
                            <div className="source-similarity">
                              Relevance: {(source.similarity_score * 100).toFixed(1)}%
                            </div>
                          </div>
                        ))}
                      </div>
                    )}
                  </div>
                )}
              </>
            )}
          </div>
        )}
      </div>
    </div>
  );
};