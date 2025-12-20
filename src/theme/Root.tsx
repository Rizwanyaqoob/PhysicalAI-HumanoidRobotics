import React from 'react';
import ClientModule from '@site/src/components/ClientModule';

// Root component that wraps the entire application
const Root = ({ children }) => {
  return (
    <>
      {children}
      <ClientModule />
    </>
  );
};

export default Root;