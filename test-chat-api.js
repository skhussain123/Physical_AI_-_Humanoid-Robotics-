// Test script to verify the chat API integration
// This simulates what the frontend component will do

const testQuestion = "What is Physical AI?";

async function testChatAPI() {
  try {
    console.log("Testing chat API integration...");
    console.log(`Sending question: "${testQuestion}"`);

    const response = await fetch('http://127.0.0.3:8001/chat', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ question: testQuestion }),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    const data = await response.json();
    console.log("API Response:", data);
    console.log("Success: Chat API is working correctly!");
  } catch (error) {
    console.error("Error testing chat API:", error);
  }
}

testChatAPI();