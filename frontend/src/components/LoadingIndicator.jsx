import React from 'react';
import './LoadingIndicator.css';

/**
 * LoadingIndicator Component
 * Shows visual feedback when the system is processing a query
 */
const LoadingIndicator = ({
  show = false,
  message = 'Processing your question...',
  showProgress = false,
  estimatedTime = null
}) => {
  if (!show) {
    return null;
  }

  return (
    <div className="loading-indicator-container">
      <div className="loading-content">
        <div className="loading-spinner">
          <div className="spinner-circle"></div>
          <div className="spinner-circle spinner-circle-2"></div>
          <div className="spinner-circle spinner-circle-3"></div>
        </div>

        <div className="loading-text">
          <p>{message}</p>
          {estimatedTime && (
            <p className="estimated-time">Estimated time: ~{estimatedTime}s</p>
          )}
        </div>

        {showProgress && (
          <div className="loading-progress">
            <div className="progress-bar">
              <div className="progress-fill"></div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default LoadingIndicator;