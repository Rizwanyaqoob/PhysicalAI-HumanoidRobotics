import React, { useEffect } from 'react';
import FloatingRAGInterface from '@site/src/theme/FloatingRAGInterface';

// Client module that gets loaded on all pages
const RAGInterfaceModule = () => {
  useEffect(() => {
    // This component will be loaded on all pages
    // The FloatingRAGInterface contains the actual UI
  }, []);

  return <FloatingRAGInterface />;
};

export default RAGInterfaceModule;