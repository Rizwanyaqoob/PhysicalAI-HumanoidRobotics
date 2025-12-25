import React, { useState } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { Chat } from '../components/Chat';

// Floating RAG Interface Component - Now using Chat interface
const FloatingRAGInterface = () => {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <div className="floating-rag-container">
      <button
        className="rag-toggle-button"
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? "Close RAG interface" : "Open RAG interface"}
      >
        {isOpen ? '✕' : '?'}
      </button>

      {isOpen && (
        <div className="rag-interface-modal">
          <div className="rag-interface-content">
            <Chat />
          </div>
        </div>
      )}

      <style jsx>{`
        .floating-rag-container {
          position: fixed;
          bottom: 20px;
          right: 20px;
          z-index: 9999 !important; /* Highest possible z-index */
        }

        .rag-toggle-button {
          width: 60px;
          height: 60px;
          border-radius: 50%;
          border: 2px solid white;
          background-color: #10a37f;
          color: white;
          font-size: 24px;
          font-weight: bold;
          cursor: pointer;
          box-shadow: 0 4px 12px rgba(0,0,0,0.4);
          display: flex;
          align-items: center;
          justify-content: center;
          transition: all 0.2s ease;
        }

        .rag-toggle-button:hover {
          transform: scale(1.1);
          box-shadow: 0 6px 16px rgba(0,0,0,0.5);
          background-color: #0d8a6a;
        }

        .rag-interface-modal {
          position: absolute;
          bottom: 60px;
          right: 0;
          width: 400px;
          max-width: calc(100vw - 40px);
          background: white;
          border-radius: 8px;
          box-shadow: 0 4px 20px rgba(0,0,0,0.3);
          overflow: hidden;
        }

        .rag-interface-content {
          padding: 0;
          max-height: 70vh;
          overflow-y: auto;
        }

        @media (max-width: 480px) {
          .rag-interface-modal {
            width: calc(100vw - 20px);
            left: 10px;
            right: 10px;
          }
        }
      `}</style>
    </div>
  );
};

const FloatingRAGInterfaceWrapper = () => {
  return (
    <BrowserOnly>
      {() => <FloatingRAGInterface />}
    </BrowserOnly>
  );
};

export default FloatingRAGInterfaceWrapper;