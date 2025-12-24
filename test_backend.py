# Test the chat functionality
import requests
import json

def test_chat():
    url = "http://127.0.0.3:8001/chat"
    data = {"question": "Introduction to Physical AI and Humanoid Robotics"}

    try:
        response = requests.post(url, json=data)
        print(f"Status Code: {response.status_code}")
        print(f"Response: {response.json()}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_chat()