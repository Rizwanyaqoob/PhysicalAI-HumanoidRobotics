import React from 'react';
import RAGInterface from '@theme/RAGInterface';

const RAGSidebar: React.FC = () => {
  return (
    <div className="rag-sidebar-container">
      <div className="rag-sidebar-content">
        <RAGInterface />
      </div>
      <style jsx>{`
        .rag-sidebar-container {
          position: fixed;
          right: 20px;
          top: 50%;
          transform: translateY(-50%);
          z-index: 1000;
          width: 350px;
        }

        @media (max-width: 996px) {
          .rag-sidebar-container {
            position: relative;
            width: 100%;
            right: 0;
            top: 0;
            transform: none;
            margin: 20px 0;
          }
        }
      `}</style>
    </div>
  );
};

export default RAGSidebar;