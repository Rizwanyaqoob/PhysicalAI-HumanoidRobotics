import React, { useState, useEffect, useRef } from 'react';
import { ChatMessage } from './ChatMessage';
import { ChatInput } from './ChatInput';
import { sendQuery, sendSelectedTextQuery } from '../frontend/src/services/query-handler';
import { sanitizeHTML } from '../frontend/src/utils/sanitizer';
import './Chat.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  sources?: Array<{
    content: string;
    source_document: string;
    page_number?: number;
    section_title?: string;
    similarity_score: number;
    chunk_id: string;
  }>;
  timestamp: Date;
}

export const Chat: React.FC = () => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const chatContainerRef = useRef<HTMLDivElement>(null);

  // Capture selected text from the page
  useEffect(() => {
    const handleSelectionChange = () => {
      const text = window.getSelection()?.toString().trim() || '';
      if (text) {
        setSelectedText(text);
      }
    };

    document.addEventListener('selectionchange', handleSelectionChange);
    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, []);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSubmit = async (input: string) => {
    if (!input.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: input,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    setError(null);

    try {
      // Add temporary assistant message with loading state
      const tempAssistantMessage: Message = {
        id: `temp-${Date.now()}`,
        role: 'assistant',
        content: '',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, tempAssistantMessage]);

      // Send query to backend
      const result = await sendQuery(input);

      // Update the temporary message with actual content and sources
      const updatedAssistantMessage: Message = {
        id: Date.now().toString(),
        role: 'assistant',
        content: result.answer,
        sources: result.sources,
        timestamp: new Date(),
      };

      setMessages(prev => {
        const newMessages = [...prev];
        // Replace the temporary message with the actual one
        const tempIndex = newMessages.findIndex(msg => msg.id.startsWith('temp-'));
        if (tempIndex !== -1) {
          newMessages[tempIndex] = updatedAssistantMessage;
        } else {
          newMessages.push(updatedAssistantMessage);
        }
        return newMessages;
      });
    } catch (err: any) {
      const errorMessage: Message = {
        id: Date.now().toString(),
        role: 'assistant',
        content: 'I couldn\'t find this in the documents. Want me to answer generally?',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
      setError(err.message || 'An error occurred while processing your query');
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      // Submit logic will be handled by the ChatInput component
    }
  };

  return (
    <div className="chat-container" onKeyDown={handleKeyDown}>
      <div className="chat-messages" ref={chatContainerRef}>
        {messages.length === 0 ? (
          <div className="chat-welcome">
            <h3>Ask anything about the book</h3>
            <p>Start a conversation with the AI assistant.</p>
          </div>
        ) : (
          messages.map((message) => (
            <ChatMessage
              key={message.id}
              message={message}
            />
          ))
        )}
        {isLoading && (
          <ChatMessage
            message={{
              id: 'loading',
              role: 'assistant',
              content: 'Thinking...',
              timestamp: new Date(),
            }}
            isStreaming={true}
          />
        )}
        <div ref={messagesEndRef} />
      </div>
      <ChatInput
        onSubmit={handleSubmit}
        isLoading={isLoading}
        selectedText={selectedText}
      />
    </div>
  );
};