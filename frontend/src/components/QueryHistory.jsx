import React from 'react';

/**
 * QueryHistory Component
 * Shows recent queries and allows navigation
 */
const QueryHistory = ({ history = [], onQuerySelect = null, maxItems = 10 }) => {
  if (!history || history.length === 0) {
    return (
      <div className="query-history-container">
        <p className="no-history">No recent queries</p>
      </div>
    );
  }

  // Limit the number of items to display
  const displayHistory = history.slice(0, maxItems);

  const handleQueryClick = (query) => {
    if (onQuerySelect && typeof onQuerySelect === 'function') {
      onQuerySelect(query);
    }
  };

  return (
    <div className="query-history-container">
      <h4 className="history-title">Recent Queries</h4>
      <div className="history-list">
        {displayHistory.map((item, index) => (
          <div key={item.id || `history-${index}`} className="history-item">
            <div className="history-query" onClick={() => handleQueryClick(item.query)}>
              <span className="query-text" title={item.query}>
                {item.query.length > 80 ? item.query.substring(0, 80) + '...' : item.query}
              </span>
              <span className="query-timestamp">{item.date}</span>
            </div>
            {item.response && (
              <div className="history-response-preview">
                <span className="response-preview" title={item.response.answer}>
                  {item.response.answer && item.response.answer.length > 100
                    ? item.response.answer.substring(0, 100) + '...'
                    : item.response.answer || 'No answer'}
                </span>
              </div>
            )}
          </div>
        ))}
      </div>
    </div>
  );
};

export default QueryHistory;