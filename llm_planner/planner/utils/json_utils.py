
"""
텍스트에서 JSON을 안전하게 추출하는 유틸리티
"""
import json

def extract_json_from_text(text: str) -> dict:
    """텍스트에서 가장 바깥 JSON 블록을 추출"""
    try:
        start = text.find('{')
        if start >= 0:
            count = 0
            for i, char in enumerate(text[start:], start):
                if char == '{':
                    count += 1
                elif char == '}':
                    count -= 1
                    if count == 0:
                        json_str = text[start:i+1]
                        return json.loads(json_str)
    except Exception:
        pass
    return {"subtasks": [], "metadata": {}}
