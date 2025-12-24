import React, { useState, useRef, useEffect } from 'react';
import './ChatWidget.css';

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'ai';
  timestamp: string; // ✅ string (ISO)
}

const API_URL = 'http://127.0.0.3:8001';
const STORAGE_KEY = 'robotics_chat_messages';

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement | null>(null);

  // ✅ Load chat from localStorage
  useEffect(() => {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved) {
      setMessages(JSON.parse(saved));
    }
  }, []);

  // ✅ Save chat to localStorage
  useEffect(() => {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(messages));
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(scrollToBottom, [messages, isLoading]);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date().toISOString(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const res = await fetch(`${API_URL}/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ question: userMessage.text }),
      });

      const data = await res.json();

      const aiMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: data.answer ?? 'No response from AI.',
        sender: 'ai',
        timestamp: new Date().toISOString(),
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (err) {
      setMessages(prev => [
        ...prev,
        {
          id: (Date.now() + 2).toString(),
          text: 'Sorry, something went wrong.',
          sender: 'ai',
          timestamp: new Date().toISOString(),
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className="chat-widget">
      {isOpen ? (
        <div className="chat-window">
          <div className="chat-header">
            <h3>AI Tutor</h3>
            <button onClick={() => setIsOpen(false)}>×</button>
          </div>

          <div className="chat-messages">
            {messages.map(msg => (
              <div key={msg.id} className={`message ${msg.sender}-message`}>
                <div className="message-content">{msg.text}</div>
                <div className="message-timestamp">
                  {new Date(msg.timestamp).toLocaleTimeString([], {
                    hour: '2-digit',
                    minute: '2-digit',
                  })}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className="message ai-message">
                <div className="message-content">Typing...</div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-area">
            <textarea
              value={inputValue}
              onChange={e => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask about robotics..."
              rows={2}
            />
            <button onClick={handleSendMessage} disabled={isLoading}>
              Send
            </button>
          </div>
        </div>
      ) : (
        <button className="chat-toggle-button" onClick={() => setIsOpen(true)}>
          💬
        </button>
      )}
    </div>
  );
};

export default ChatWidget;
