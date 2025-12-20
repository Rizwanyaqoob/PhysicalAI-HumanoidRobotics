import React, { useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import FloatingRAGInterface from '@site/src/theme/FloatingRAGInterface';

// Client module to inject the floating RAG interface
const ClientModule = () => {
  return (
    <BrowserOnly>
      {() => <FloatingRAGInterface />}
    </BrowserOnly>
  );
};

export default ClientModule;