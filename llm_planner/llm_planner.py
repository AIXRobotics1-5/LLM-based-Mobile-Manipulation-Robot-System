import json
import os
from google import genai
from google.genai import types

client = genai.Client(api_key=os.environ.get("GOOGLE_API_KEY"))

SYSTEM_PROMPT = """
당신은 스마트 팩토리의 물류 판단 AI입니다.
사용자의 자연어 명령을 분석하여, 실행 가능하면 작업을 생성하고, 애매하면 질문하세요.

## 구역별 역할 및 색상 분류
- A구역: 적재 (빨간색 전용)
- B구역: 적재 (초록색 전용)
- C구역: 적재 (노란색 전용)
- D구역: 출고/포장

## 구역 내 세부 위치
- front (앞/앞쪽/바깥) / center (중앙/가운데) / back (안/안쪽/뒤)

## 물품 분류: 빨간색→A, 초록색→B, 노란색→C

## 세트 정의
- a세트: 빨간색 1 + 초록색 1
- b세트: 초록색 1 + 노란색 1
- c세트: 빨간색 2

## 작업 순서 규칙
- 여러 색상의 작업이 동시에 있을 경우, tasks 배열을 반드시 빨간색(red) → 노란색(yellow) → 초록색(green) 순서로 정렬하세요.

## 애매하지 않은 경우 (규칙으로 해소 가능 → 바로 실행)
- "정리해줘" → 색상별 분류
- "출고해줘" → D구역 이동
- 색상만 지정, 구역 미지정 → 색상 분류 규칙 적용
- 수량 미지정 → "all"
- 위치 미지정 → "auto" (후처리에서 자동 배분)

## 사용자 확인/승인 처리
- 사용자가 이전 질문에 대해 "네", "응", "1", "확인", "옮겨줘", "해줘" 등으로 승인하면, 같은 질문을 반복하지 말고 바로 execute로 실행하세요.
- 지정 구역이 아닌 다른 구역으로 옮기라는 명령을 사용자가 명시적으로 했거나 확인했다면, 추가 질문 없이 바로 실행하세요.
- 같은 질문을 두 번 이상 하지 마세요.

## 애매한 경우 (질문 필요)
1. 대상 불명확: "블록 옮겨줘" (어떤 색상?)
2. 목적지 불명확: 규칙으로도 해소 안 되는 경우
3. 다수 해석 가능: "저쪽에 놔줘"
4. 수량 충돌: "하나만 옮겨줘" (여러 색상 중 어떤 것?)
5. 실행 불가능: 존재하지 않는 구역/색상
6. 명령 모호: 위 규칙으로도 판단 불가

## 응답 형식 (JSON만 반환)

### 명확한 경우:
{
  "action": "execute",
  "reasoning": "판단 근거",
  "tasks": [
    {
      "color": "red / yellow / green",
      "count": 숫자 또는 "all",
      "to_zone": "A / B / C / D",
      "to_position": "front / center / back"
    }
  ]
}

### 애매한 경우:
{
  "action": "ask",
  "reasoning": "왜 애매한지",
  "question": "사용자에게 할 질문 (한국어, 짧고 친절하게)",
  "options": ["선택지1", "선택지2", ...] 또는 null
}
"""


def _parse_json(text: str):
  lines = text.splitlines()
  for i, line in enumerate(lines):
    if line.strip() == "```json":
      text = "\n".join(lines[i + 1:])
      text = text.split("```")[0]
      break
  return text


def _check_demo(user_input: str):
  """시연용 고정 시나리오 매칭"""
  try:
    with open("demo_scenarios.json", "r", encoding="utf-8") as f:
      demos = json.load(f)
  except FileNotFoundError:
    return None

  for scenario in demos.get("scenarios", []):
    keywords = scenario.get("keywords", [])
    if all(kw in user_input for kw in keywords):
      return scenario["output"]
  return None


def process_command(user_input: str) -> dict:
  """사용자 명령 → 실행 or 질문 (시연 시나리오 우선 매칭)"""
  demo = _check_demo(user_input)
  if demo:
    return demo

  prompt = f"""
## 사용자 명령
"{user_input}"
"""

  config = types.GenerateContentConfig(
    temperature=0,
    system_instruction=SYSTEM_PROMPT,
    thinking_config=types.ThinkingConfig(thinking_budget=2048)
  )

  response = client.models.generate_content(
    model="gemini-2.5-flash",
    contents=[prompt],
    config=config
  )

  result = json.loads(_parse_json(response.text))

  if result.get("action") == "execute":
    result["tasks"] = _sort_by_color(result.get("tasks", []))
    result["tasks"] = _assign_positions(result.get("tasks", []))

  return result


POSITION_ORDER = ["front", "center", "back"]
COLOR_PRIORITY = {"red": 0, "yellow": 1, "green": 2}


def _sort_by_color(tasks):
  """빨강 → 노랑 → 초록 순으로 정렬"""
  return sorted(tasks, key=lambda t: COLOR_PRIORITY.get(t.get("color"), 99))


def _assign_positions(tasks):
  """count가 2 이상이고 위치 미지정이면, task를 개별로 펼쳐서 front→center→back 순 배분"""
  expanded = []
  zone_counters = {}

  for task in tasks:
    zone = task.get("to_zone")
    pos = task.get("to_position", "auto")
    count = task.get("count", 1)

    # 위치가 지정된 경우 그대로
    if pos not in ("auto", None):
      expanded.append(task)
      continue

    # count가 all이거나 1이면 center
    if count == "all" or count == 1:
      task["to_position"] = "center"
      expanded.append(task)
      continue

    # count >= 2, 위치 미지정 → 개별로 펼쳐서 배분
    for i in range(count):
      idx = zone_counters.get(zone, 0)
      expanded.append({
        **task,
        "count": 1,
        "to_position": POSITION_ORDER[idx % len(POSITION_ORDER)],
      })
      zone_counters[zone] = idx + 1

  return expanded
