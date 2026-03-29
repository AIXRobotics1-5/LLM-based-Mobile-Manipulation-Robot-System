import json
from llm_planner import process_command

COLOR_KR = {"red": "빨간색", "yellow": "노란색", "green": "초록색"}
POS_KR = {"front": "바깥쪽", "center": "중앙", "back": "안쪽"}


def print_tasks(tasks):
  print(f"\n작업 {len(tasks)}건:")
  for t in tasks:
    color = COLOR_KR.get(t["color"], t["color"])
    pos = POS_KR.get(t.get("to_position", "center"), "중앙")
    count = t.get("count", "all")
    count_str = "전부" if count == "all" else f"{count}개"
    print(f"  - {color} 블록 {count_str} → {t['to_zone']}구역 ({pos})")


def main():
  print("올인원 ")
  print("명령 예시: '빨간 블록 A구역에 넣어줘', '정리해줘', 'a세트 포장해줘'\n")

  context = ""
  pending_followup = None  # 데모 시나리오 후속 응답

  while True:
    user_input = input("명령 (종료: q): ").strip()
    if user_input.lower() == "q":
      break
    if not user_input:
      continue

    try:
      # 데모 시나리오 후속 응답 처리
      if pending_followup:
        answer = user_input.strip()
        result = pending_followup.get(answer)
        pending_followup = None
        context = ""

        if result:
          print(f"\n[판단] {result.get('reasoning', '')}")
          if result.get("tasks"):
            print_tasks(result["tasks"])
            with open("output.json", "w", encoding="utf-8") as f:
              json.dump(result, f, indent=2, ensure_ascii=False)
            print(f"작업 JSON 저장: output.json")
          continue
        # 매칭 안 되면 LLM으로 폴백

      # 이전 질문에 대한 답변이면 맥락 합치기
      if context:
        combined = f"이전 명령: {context}\n사용자 답변: {user_input}"
      else:
        combined = user_input

      print("LLM 판단 중...")
      result = process_command(combined)

      if result.get("action") == "execute":
        context = ""
        print(f"\n[판단] {result.get('reasoning', '')}")
        if result.get("tasks"):
          print_tasks(result["tasks"])
          with open("output.json", "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
          print(f"작업 JSON 저장: output.json")
        else:
          print("수행할 작업이 없습니다.")

      elif result.get("action") == "ask":
        context = combined
        # 데모 시나리오의 followup이 있으면 저장
        pending_followup = result.get("followup")

        print(f"\n[질문] {result.get('question', '')}")
        options = result.get("options")
        if options:
          for i, opt in enumerate(options, 1):
            print(f"  {i}. {opt}")
          print("  (번호 또는 직접 입력)")

    except Exception as e:
      print(f"오류 발생: {e}")
      context = ""
      pending_followup = None

  print("종료합니다.")


if __name__ == "__main__":
  main()
