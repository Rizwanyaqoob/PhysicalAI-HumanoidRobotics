import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import { Chat } from '../components/Chat';

// RAGInterface - Now using Chat interface
const RAGInterfaceWrapper: React.FC = () => {
  return (
    <div className="rag-interface-container">
      <Chat />
    </div>
  );
};

const RAGInterfaceWithBrowserOnly: React.FC = () => {
  return (
    <BrowserOnly>
      {() => <RAGInterfaceWrapper />}
    </BrowserOnly>
  );
};

export default RAGInterfaceWithBrowserOnly;