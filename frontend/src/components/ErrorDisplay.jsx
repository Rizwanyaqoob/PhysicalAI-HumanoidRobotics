  import React from 'react';
  import './ErrorDisplay.css';

  /**
   * ErrorDisplay Component
   * Shows error messages with appropriate styling for different error types
   */
  const ErrorDisplay = ({
    error,
    showError = false,
    onRetry = null,
    showRetryButton = true,
    errorType = 'general' // 'network', 'backend', 'validation', 'empty', 'general'
  }) => {
    if (!showError || !error) {
      return null;
    }

    // Determine error type and message based on the error object
    let displayMessage = 'An error occurred';
    let displayType = errorType;
    let icon = '⚠️';
    let title = 'Error';

    // If error is a string, use it directly
    if (typeof error === 'string') {
      displayMessage = error;
    } else if (error && typeof error === 'object') {
      // Handle different error object structures
      if (error.message) {
        displayMessage = error.message;
      } else if (error.error) {
        displayMessage = error.error;
      } else {
        displayMessage = JSON.stringify(error);
      }

      // Determine error type from the message content
      if (displayMessage.toLowerCase().includes('network') ||
          displayMessage.toLowerCase().includes('fetch') ||
          displayMessage.toLowerCase().includes('connect')) {
        displayType = 'network';
      } else if (displayMessage.toLowerCase().includes('backend') ||
                displayMessage.toLowerCase().includes('server') ||
                displayMessage.toLowerCase().includes('500') ||
                displayMessage.toLowerCase().includes('internal server')) {
        displayType = 'backend';
      } else if (displayMessage.toLowerCase().includes('empty') ||
                displayMessage.toLowerCase().includes('no relevant content')) {
        displayType = 'empty';
      } else if (displayMessage.toLowerCase().includes('validation') ||
                displayMessage.toLowerCase().includes('bad request') ||
                displayMessage.toLowerCase().includes('400')) {
        displayType = 'validation';
      }
    }

    // Set specific messages and styling based on error type
    switch (displayType) {
      case 'network':
        icon = '🔌';
        title = 'Network Error';
        if (!displayMessage.includes('Unable to connect')) {
          displayMessage = 'Unable to connect to the question-answering service. Please check your connection and try again.';
        }
        break;
      case 'backend':
        icon = '🔧';
        title = 'Service Error';
        if (!displayMessage.includes('temporarily unavailable')) {
          displayMessage = 'The question-answering service is temporarily unavailable. Please try again in a moment.';
        }
        break;
      case 'empty':
        icon = '🔍';
        title = 'No Results';
        if (!displayMessage.includes('No relevant content')) {
          displayMessage = 'No relevant content found in the book for your question. Try rephrasing or asking about a different topic.';
        }
        break;
      case 'validation':
        icon = '❌';
        title = 'Invalid Request';
        break;
      default:
        icon = '⚠️';
        title = 'Error';
    }

    const handleRetry = () => {
      if (onRetry) {
        onRetry();
      }
    };

    return (
      <div className={`error-display-container error-type-${displayType}`}>
        <div className="error-content">
          <div className="error-header">
            <span className="error-icon">{icon}</span>
            <h4 className="error-title">{title}</h4>
          </div>
          <p className="error-message">{displayMessage}</p>
          {showRetryButton && onRetry && (
            <button className="retry-button" onClick={handleRetry}>
              Try Again
            </button>
          )}
        </div>
      </div>
    );
  };

  export default ErrorDisplay;