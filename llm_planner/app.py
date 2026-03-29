import json
import threading
from flask import Flask, request, jsonify, send_from_directory
from llm_planner import process_command

# ROS2 publisher for auto tasks
import rclpy
from std_msgs.msg import String

rclpy.init()
_ros2_node = rclpy.create_node("llm_app_bridge")
_auto_task_pub = _ros2_node.create_publisher(String, "/dsr01/curobo/auto_task", 10)
_sim_cmd_pub = _ros2_node.create_publisher(String, "/digital_twin/command", 10)

# Spin ROS2 in background thread
_spin_thread = threading.Thread(
    target=lambda: rclpy.spin(_ros2_node), daemon=True
)
_spin_thread.start()


def publish_tasks(tasks):
    """Publish task list to /dsr01/curobo/auto_task for robot execution.
    Also sends appropriate Isaac Sim commands based on task content.
    """
    # Determine Isaac Sim command based on tasks
    zones_in_tasks = set(t.get("to_zone", "") for t in tasks)
    colors_in_tasks = set(t.get("color", "") for t in tasks)

    if "D" in zones_in_tasks:
        # 출고 task → move AMR to relevant zone based on color
        # red→A, green→B, yellow→C
        color_to_zone = {"red": "A", "green": "B", "yellow": "C"}
        for color in colors_in_tasks:
            amr_zone = color_to_zone.get(color)
            if amr_zone:
                send_sim_command(f"work {amr_zone}")
                break  # one zone at a time
    elif zones_in_tasks & {"A", "B", "C"}:
        # 입고/분류 task → AMR should be at inbound (conveyor)
        send_sim_command("inbound")

    msg = String()
    msg.data = json.dumps(tasks)
    _auto_task_pub.publish(msg)
    print(f"[LLM→ROS2] Published {len(tasks)} tasks")


def send_sim_command(cmd):
    """Send command to Isaac Sim digital twin."""
    msg = String()
    msg.data = cmd
    _sim_cmd_pub.publish(msg)
    print(f"[LLM→Sim] {cmd}")


from flask import Response
import time as _time
import os as _os

app = Flask(__name__, static_folder="static")

COLOR_KR = {"red": "빨간색", "yellow": "노란색", "green": "초록색"}
POS_KR = {"front": "바깥쪽", "center": "중앙", "back": "안쪽"}

# 대화 상태
context = ""
pending_followup = None


def format_tasks(tasks):
    lines = [f"<strong>작업 {len(tasks)}건:</strong>"]
    for t in tasks:
        color = COLOR_KR.get(t["color"], t["color"])
        pos = POS_KR.get(t.get("to_position", "center"), "중앙")
        count = t.get("count", "all")
        count_str = "전부" if count == "all" else f"{count}개"
        badge_class = t["color"]
        lines.append(
            f'<div class="task-item">'
            f'<span class="color-badge {badge_class}"></span>'
            f'{color} 블록 {count_str} → {t["to_zone"]}구역 ({pos})'
            f'</div>'
        )
    return "\n".join(lines)


def format_ask(result):
    msg = f"<strong>{result.get('question', '')}</strong>"
    options = result.get("options")
    if options:
        msg += '<div class="options">'
        for i, opt in enumerate(options, 1):
            msg += f'<button class="option-btn" onclick="sendOption(\'{i}\')">{i}. {opt}</button>'
        msg += '</div>'
        msg += '<div class="hint">번호 또는 직접 입력</div>'
    return msg


@app.route("/")
def index():
    return send_from_directory("static", "index.html")


@app.route("/api/chat", methods=["POST"])
def chat():
    global context, pending_followup

    data = request.get_json()
    user_input = data.get("message", "").strip()
    if not user_input:
        return jsonify({"error": "빈 메시지"}), 400

    try:
        # 데모 시나리오 후속 응답 처리
        if pending_followup:
            answer = user_input.strip()
            result = pending_followup.get(answer)
            pending_followup = None
            context = ""

            if result:
                msg = f"<div class='reasoning'>{result.get('reasoning', '')}</div>"
                if result.get("tasks"):
                    msg += format_tasks(result["tasks"])
                    with open("output.json", "w", encoding="utf-8") as f:
                        json.dump(result, f, indent=2, ensure_ascii=False)
                    publish_tasks(result["tasks"])
                return jsonify({"reply": msg, "type": "execute"})

        # 이전 질문에 대한 답변이면 맥락 합치기
        if context:
            combined = f"이전 명령: {context}\n사용자 답변: {user_input}"
        else:
            combined = user_input

        result = process_command(combined)

        if result.get("action") == "execute":
            context = ""
            msg = f"<div class='reasoning'>{result.get('reasoning', '')}</div>"
            if result.get("tasks"):
                msg += format_tasks(result["tasks"])
                with open("output.json", "w", encoding="utf-8") as f:
                    json.dump(result, f, indent=2, ensure_ascii=False)
                publish_tasks(result["tasks"])
            else:
                msg += "<p>수행할 작업이 없습니다.</p>"
            return jsonify({"reply": msg, "type": "execute"})

        elif result.get("action") == "ask":
            context = combined
            pending_followup = result.get("followup")
            return jsonify({"reply": format_ask(result), "type": "ask"})

    except Exception as e:
        context = ""
        pending_followup = None
        return jsonify({"reply": f"오류 발생: {e}", "type": "error"}), 500


def _gen_mjpeg(path):
    """Generator that yields MJPEG frames from a file on disk."""
    while True:
        if _os.path.exists(path):
            try:
                with open(path, "rb") as f:
                    frame = f.read()
                if frame:
                    yield (b"--frame\r\n"
                           b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
            except Exception:
                pass
        _time.sleep(0.2)  # ~5 fps (stable streaming)


@app.route("/stream/curobo")
def stream_curobo():
    """MJPEG stream of curobo_vision camera feed."""
    return Response(_gen_mjpeg("/tmp/curobo_frame.jpg"),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/stream/isaacsim")
def stream_isaacsim():
    """MJPEG stream of Isaac Sim viewport."""
    return Response(_gen_mjpeg("/tmp/isaacsim_frame.jpg"),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=7860, debug=False)
