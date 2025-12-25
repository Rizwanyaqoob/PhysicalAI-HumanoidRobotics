import React from 'react';
import './ChatStatus.css';

type StatusType = 'loading' | 'error' | 'success';

interface ChatStatusProps {
  status: StatusType;
  message: string;
  onRetry?: () => void;
  showRetryButton?: boolean;
  estimatedTime?: number;
}

/**
 * ChatStatus Component
 * Shows loading, error, or other status messages
 */
export const ChatStatus: React.FC<ChatStatusProps> = ({
  status,
  message,
  onRetry,
  showRetryButton = false,
  estimatedTime
}) => {
  let displayMessage = message;
  let displayType = status;
  let icon = '⚠️';
  let title = 'Status';

  // Set specific messages and styling based on status type
  switch (status) {
    case 'loading':
      icon = '⏳';
      title = 'Processing';
      break;
    case 'error':
      icon = '❌';
      title = 'Error';
      break;
    case 'success':
      icon = '✅';
      title = 'Success';
      break;
    default:
      icon = 'ℹ️';
      title = 'Info';
  }

  const handleRetry = () => {
    if (onRetry) {
      onRetry();
    }
  };

  return (
    <div className={`chat-status-container chat-status-${status}`}>
      <div className="chat-status-content">
        <div className="chat-status-header">
          <span className="chat-status-icon">{icon}</span>
          <h4 className="chat-status-title">{title}</h4>
        </div>
        <p className="chat-status-message">{displayMessage}</p>
        {status === 'loading' && estimatedTime && (
          <p className="estimated-time">Estimated time: ~{estimatedTime}s</p>
        )}
        {showRetryButton && onRetry && (
          <button className="retry-button" onClick={handleRetry}>
            Try Again
          </button>
        )}
      </div>
    </div>
  );
};