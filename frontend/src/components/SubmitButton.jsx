import React from 'react';
import './SubmitButton.css';

/**
 * SubmitButton Component
 * A reusable button component for submitting queries
 */
const SubmitButton = ({
  onClick,
  disabled = false,
  text = 'Submit',
  loading = false,
  className = ''
}) => {
  const handleClick = (e) => {
    if (!disabled && onClick) {
      onClick(e);
    }
  };

  return (
    <button
      type="button"
      onClick={handleClick}
      disabled={disabled || loading}
      className={`submit-button ${className} ${loading ? 'loading' : ''}`}
    >
      {loading ? (
        <span className="loading-spinner">
          <span className="spinner-icon">⏳</span> {text}...
        </span>
      ) : (
        text
      )}
    </button>
  );
};

export default SubmitButton;