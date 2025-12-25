import React from 'react';
import { Chat } from './Chat';
import './RAGChat.css';

interface RAGChatProps {
  variant?: 'floating' | 'sidebar' | 'inline'; // Different UI variants
  className?: string; // Additional CSS classes
  placeholder?: string; // Input placeholder text
  title?: string; // Chat title
}

/**
 * Core RAGChat Component (now using Chat interface)
 * Handles all the logic for RAG-based chat functionality
 */
export const RAGChat: React.FC<RAGChatProps> = ({
  variant = 'sidebar',
  className = '',
  placeholder,
  title
}) => {
  return (
    <div className={`rag-chat-container rag-chat-${variant} ${className}`}>
      <Chat />
    </div>
  );
};

export default RAGChat;