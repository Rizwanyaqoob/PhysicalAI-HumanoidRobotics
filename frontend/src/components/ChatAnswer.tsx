import React from 'react';
import { sanitizeHTML } from '../utils/sanitizer';
import './ChatAnswer.css';

interface ChatAnswerProps {
  answer: string;
}

/**
 * ChatAnswer Component
 * Displays the answer from the RAG agent
 */
export const ChatAnswer: React.FC<ChatAnswerProps> = ({ answer }) => {
  // Sanitize the answer content
  const sanitizedAnswer = sanitizeHTML(answer || '');

  return (
    <div className="chat-answer-container">
      <div className="chat-answer-header">
        <h3>Answer</h3>
      </div>

      <div className="chat-answer-content">
        {sanitizedAnswer ? (
          <div
            className="answer-text"
            dangerouslySetInnerHTML={{ __html: sanitizedAnswer }}
          />
        ) : (
          <p className="no-answer">No answer provided.</p>
        )}
      </div>
    </div>
  );
};