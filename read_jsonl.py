import json
import sys

def parse_jsonl():
    with open('d:\\SLAM Bot\\5ea92256-2474-47fd-88c9-9b572f9f64b0.jsonl', 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    for line in lines[-200:]:
        try:
            data = json.loads(line)
            if isinstance(data, list):
                # sometimes array of messages
                for msg in data:
                    if isinstance(msg, dict) and 'role' in msg:
                        content = msg.get('content', '')
                        if isinstance(content, list):
                            content = ' '.join(c.get('text', '') for c in content if isinstance(c, dict) and 'text' in c)
                        print(f"{msg.get('role')}: {str(content)[:1000]}")
            elif isinstance(data, dict):
                # Check for standard claude code transcript format
                if data.get('type') in ['message', 'assistant', 'user']:
                    print(f"{data.get('type')}: {data.get('message', {}).get('content') or data.get('content')}")
                    continue
                if 'message' in data and isinstance(data['message'], dict):
                    msg = data['message']
                    role = msg.get('role')
                    content = msg.get('content', '')
                    if isinstance(content, list):
                        content = ' '.join(c.get('text', '') for c in content if isinstance(c, dict) and 'text' in c)
                    if role:
                        print(f"{role}: {str(content)[:1000]}")
        except Exception as e:
            pass

if __name__ == '__main__':
    parse_jsonl()
