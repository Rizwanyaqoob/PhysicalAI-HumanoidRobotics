import React from 'react';
import { Chat } from '../../src/components/Chat';
import './RAGInterface.css';

/**
 * RAGInterface - Now using Chat component
 * Maintains backward compatibility while using the new architecture
 */
const RAGInterface = () => {
  return (
    <div className="rag-interface-container">
      <Chat />
    </div>
  );
};

export default RAGInterface;