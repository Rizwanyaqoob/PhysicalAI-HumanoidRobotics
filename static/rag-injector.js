// Simple RAG Interface Injector - Pure JavaScript approach

function initializeRAGInterface() {
  console.log('Initializing RAG Interface...');

  // Create the floating button
  const button = document.createElement('div');
  button.id = 'rag-interface-button';
  button.innerHTML = '?';
  button.style.position = 'fixed';
  button.style.bottom = '20px';
  button.style.right = '20px';
  button.style.width = '60px';
  button.style.height = '60px';
  button.style.borderRadius = '50%';
  button.style.border = '2px solid white';
  button.style.backgroundColor = '#3578e5';
  button.style.color = 'white';
  button.style.fontSize = '24px';
  button.style.fontWeight = 'bold';
  button.style.cursor = 'pointer';
  button.style.boxShadow = '0 4px 12px rgba(0,0,0,0.4)';
  button.style.display = 'flex';
  button.style.alignItems = 'center';
  button.style.justifyContent = 'center';
  button.style.zIndex = '2147483647'; // Maximum z-index to ensure visibility
  button.style.transition = 'all 0.2s ease';
  button.style.userSelect = 'none';
  // Visual indicator to ensure visibility
  button.style.border = '2px solid yellow';
  button.style.boxShadow = '0 4px 12px rgba(0,0,0,0.4), 0 0 15px rgba(255,255,0,0.7)';

  // Add hover effects
  button.addEventListener('mouseenter', function() {
    this.style.transform = 'scale(1.1)';
    this.style.boxShadow = '0 6px 16px rgba(0,0,0,0.5)';
    this.style.backgroundColor = '#2a64c5';
  });

  button.addEventListener('mouseleave', function() {
    this.style.transform = 'scale(1)';
    this.style.boxShadow = '0 4px 12px rgba(0,0,0,0.4)';
    this.style.backgroundColor = '#3578e5';
  });

  // Add click handler
  let isOpen = false;
  button.addEventListener('click', function() {
    toggleRAGInterface();
  });

  // Create the interface modal
  const modal = document.createElement('div');
  modal.id = 'rag-interface-modal';
  modal.style.display = 'none';
  modal.style.position = 'fixed';
  modal.style.bottom = '80px';
  modal.style.right = '20px';
  modal.style.width = '400px';
  modal.style.maxWidth = 'calc(100vw - 40px)';
  modal.style.backgroundColor = 'white';
  modal.style.borderRadius = '8px';
  modal.style.boxShadow = '0 4px 20px rgba(0,0,0,0.3)';
  modal.style.zIndex = '9998';
  modal.style.overflow = 'hidden';

  // Create modal content
  modal.innerHTML = `
    <div style="padding: 20px; max-height: 70vh; overflow-y: auto;">
      <h3 style="margin: 0 0 16px 0; color: #2c3e50;">Ask Questions About the Book</h3>

      <form id="rag-query-form" style="margin-bottom: 16px;">
        <div style="margin-bottom: 12px;">
          <textarea
            id="rag-query-input"
            placeholder="Ask a question about the book content..."
            style="width: 100%; min-height: 80px; padding: 8px; border: 1px solid #ccc; border-radius: 4px; resize: vertical;"
          ></textarea>
        </div>

        <div id="selected-text-info" style="margin-bottom: 12px; display: none;">
          <label style="display: flex; align-items: center;">
            <input type="checkbox" id="use-selected-text" style="margin-right: 8px;">
            Restrict to selected text: <em id="selected-text-preview"></em>
          </label>
        </div>

        <button
          type="submit"
          id="rag-submit-button"
          style="background-color: #3578e5; color: white; border: none; padding: 8px 16px; border-radius: 4px; cursor: pointer;"
        >
          Ask Question
        </button>
      </form>

      <div id="rag-loading" style="display: none; padding: 16px; text-align: center; color: #666;">
        <div>Processing your question...</div>
        <div style="margin-top: 8px;">
          <div style="display: inline-block; width: 20px; height: 20px; border: 3px solid #f3f3f3; border-top: 3px solid #3578e5; border-radius: 50%; animation: spin 1s linear infinite;"></div>
          <style>
            @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
          </style>
        </div>
      </div>

      <div id="rag-error" style="display: none; padding: 12px; background-color: #ffebee; border: 1px solid #ffcdd2; border-radius: 4px; color: #c62828; margin: 12px 0;">
        <div><strong>Error:</strong> <span id="error-message"></span></div>
        <button id="retry-button" style="margin-top: 8px; padding: 4px 8px; background-color: #3578e5; color: white; border: none; border-radius: 4px; cursor: pointer;">Retry</button>
      </div>

      <div id="rag-response" style="display: none; margin-top: 16px; padding: 16px; background-color: white; border: 1px solid #e0e0e0; border-radius: 4px;">
        <h4 style="margin: 0 0 12px 0; color: #2c3e50;">Answer</h4>
        <div id="answer-content" style="white-space: pre-wrap; line-height: 1.6;"></div>
        <div id="sources-container" style="margin-top: 16px; display: none;">
          <h5 style="margin: 0 0 8px 0; color: #555;">Sources:</h5>
          <ul id="sources-list" style="margin: 8px 0; padding-left: 20px;"></ul>
        </div>
        <div id="timing-info" style="margin-top: 12px; font-size: 0.8em; color: #777; border-top: 1px solid #eee; padding-top: 8px; display: none;"></div>
      </div>
    </div>
  `;

  // Add to document
  document.body.appendChild(button);
  document.body.appendChild(modal);

  console.log('RAG Interface button and modal added to document body');
  console.log('Button element:', document.getElementById('rag-interface-button'));

  // Track selected text
  let selectedText = '';

  document.addEventListener('selectionchange', function() {
    const selection = window.getSelection();
    if (selection.toString().trim()) {
      selectedText = selection.toString().trim();
      const previewElement = document.getElementById('selected-text-preview');
      const infoElement = document.getElementById('selected-text-info');
      const checkbox = document.getElementById('use-selected-text');

      if (selectedText) {
        previewElement.textContent = selectedText.substring(0, 100) + (selectedText.length > 100 ? '...' : '');
        infoElement.style.display = 'block';
        checkbox.checked = true; // Default to using selected text
      } else {
        infoElement.style.display = 'none';
        checkbox.checked = false;
      }
    }
  });

  // Form submission handler
  document.getElementById('rag-query-form').addEventListener('submit', async function(e) {
    e.preventDefault();

    const queryInput = document.getElementById('rag-query-input');
    const query = queryInput.value.trim();
    if (!query) return;

    const useSelected = document.getElementById('use-selected-text').checked;
    const finalQuery = useSelected && selectedText ? `Based on the following text: "${selectedText}", ${query}` : query;

    // Show loading, hide error and response
    document.getElementById('rag-loading').style.display = 'block';
    document.getElementById('rag-error').style.display = 'none';
    document.getElementById('rag-response').style.display = 'none';

    try {
      const backendUrl = window.BACKEND_API_URL || 'http://localhost:8009';
      const response = await fetch(`${backendUrl}/api/ask`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: finalQuery,
          provider: 'gemini',
          max_chunks: 5
        })
      });

      if (!response.ok) {
        throw new Error(`Backend error: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();

      // Display response
      document.getElementById('answer-content').textContent = data.answer;

      // Display sources if available
      const sourcesContainer = document.getElementById('sources-container');
      const sourcesList = document.getElementById('sources-list');
      sourcesList.innerHTML = '';

      if (data.sources && data.sources.length > 0) {
        data.sources.forEach((source, index) => {
          const li = document.createElement('li');
          li.style.marginBottom = '8px';
          li.innerHTML = `
            <strong>${source.source_document}</strong>
            ${source.page_number ? `, Page: ${source.page_number}` : ''}
            ${source.section_title ? `, Section: ${source.section_title}` : ''}
            <div style="font-size: 0.9em; color: #666; margin-top: 4px;">
              ${source.content.substring(0, 200)}${source.content.length > 200 ? '...' : ''}
            </div>
          `;
          sourcesList.appendChild(li);
        });
        sourcesContainer.style.display = 'block';
      } else {
        sourcesContainer.style.display = 'none';
      }

      // Display timing info if available
      const timingInfo = document.getElementById('timing-info');
      if (data.debug_info) {
        timingInfo.textContent = `Provider: ${data.providerUsed} | Total time: ${data.debug_info.total_time?.toFixed(2)}s`;
        timingInfo.style.display = 'block';
      } else {
        timingInfo.style.display = 'none';
      }

      document.getElementById('rag-response').style.display = 'block';
    } catch (error) {
      console.error('Error processing query:', error);
      document.getElementById('error-message').textContent = error.message || 'An error occurred while processing your query';
      document.getElementById('rag-error').style.display = 'block';
    } finally {
      document.getElementById('rag-loading').style.display = 'none';
    }
  });

  // Retry button handler
  document.getElementById('retry-button').addEventListener('click', function() {
    document.getElementById('rag-query-form').dispatchEvent(new Event('submit'));
  });

  // Toggle function
  window.toggleRAGInterface = function() {
    isOpen = !isOpen;
    const modal = document.getElementById('rag-interface-modal');
    const button = document.getElementById('rag-interface-button');

    if (isOpen) {
      modal.style.display = 'block';
      button.innerHTML = '✕';
      button.style.backgroundColor = '#e53e3e';
    } else {
      modal.style.display = 'none';
      button.innerHTML = '?';
      button.style.backgroundColor = '#3578e5';
    }
  };

  // Close modal when clicking outside
  document.addEventListener('click', function(e) {
    const modal = document.getElementById('rag-interface-modal');
    const button = document.getElementById('rag-interface-button');

    if (isOpen &&
        !modal.contains(e.target) &&
        e.target !== button &&
        !e.target.closest('#rag-interface-modal')) {
      isOpen = false;
      modal.style.display = 'none';
      button.innerHTML = '?';
      button.style.backgroundColor = '#3578e5';
    }
  });
}

// Initialize when document is ready - handle async loading scenario
console.log('RAG Interface script loaded, document readyState:', document.readyState);
if (document.readyState === 'loading') {
  // Document still loading, wait for DOMContentLoaded
  console.log('Waiting for DOMContentLoaded to initialize RAG interface');
  document.addEventListener('DOMContentLoaded', initializeRAGInterface);
} else if (document.readyState === 'interactive' || document.readyState === 'complete') {
  // Document already loaded, initialize immediately
  console.log('Initializing RAG interface immediately');
  initializeRAGInterface();
} else {
  // Fallback for any other state
  console.log('Unknown readyState, waiting for DOMContentLoaded');
  document.addEventListener('DOMContentLoaded', initializeRAGInterface);
}