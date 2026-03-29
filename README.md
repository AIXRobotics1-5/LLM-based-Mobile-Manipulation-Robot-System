# LLM-based Mobile Manipulation Robot System

> **제 1회 AI x Robotics 융합 해커톤 — 5팀**
> 일시: 2026.03.21 (토) | 장소: 서울 성수 CF타워

AI 비전과 LLM을 활용한 Mobile Manipulation 로봇 시스템입니다.
자연어 명령으로 물류 입출고 작업을 수행하며, Isaac Sim 기반 디지털 트윈으로 실시간 모니터링합니다.

## Team

| 이름 | GitHub |
|------|--------|
| 김민준 | [@fhekwn549](https://github.com/fhekwn549) |
| 우지민 | [@jim2ning](https://github.com/jim2ning) |
| 최민호 | [@minhochoe66](https://github.com/minhochoe66) |

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        User Interface                           │
│              Gradio Web App (localhost:7860)                     │
│                   자연어 명령 입력                                │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                     LLM Task Planner                            │
│            Google Gemini 2.5 Flash 기반                          │
│      자연어 → JSON 태스크 변환 (색상/구역/위치)                    │
└──────────┬───────────────────────────────────┬──────────────────┘
           │                                   │
           ▼                                   ▼
┌─────────────────────┐             ┌─────────────────────────────┐
│   ROS2 Topic Pub    │             │   Isaac Sim Command Pub     │
│ /dsr01/curobo/      │             │ /digital_twin/command       │
│    auto_task        │             │  (inbound/work A/outbound)  │
└──────────┬──────────┘             └──────────┬──────────────────┘
           │                                   │
           ▼                                   ▼
┌─────────────────────┐             ┌─────────────────────────────┐
│  Vision + Planning  │             │      ZMQ Bridge (5557)      │
│  (e0509_gripper_    │────────────▶│  ROS2 → Isaac Sim 동기화     │
│   description)      │             │  joints / gripper / objects  │
│                     │             └──────────┬──────────────────┘
│ • Grounding DINO    │                        │
│   제로샷 물체 인식   │                        ▼
│ • RealSense D455F   │             ┌─────────────────────────────┐
│   Depth 3D 위치     │             │    Isaac Sim Digital Twin    │
│ • cuRobo GPU 가속   │             │ • Carter AMR 시뮬레이션      │
│   모션 플래닝 ~70ms │             │ • E0509 로봇팔 미러링        │
│ • Eye-to-Hand       │             │ • 공장 레이아웃 시각화        │
│   캘리브레이션       │             └─────────────────────────────┘
└──────────┬──────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Physical Robot                               │
│         Doosan E0509 (6축) + RH-P12-RN-A Gripper                │
│              Modbus RTU 그리퍼 제어 / MoveJ, MoveL               │
└─────────────────────────────────────────────────────────────────┘
```

## Process Flow

### 입고 시나리오
```
사용자: "빨간색 물건 1개 A구역 안쪽에 보관해줘"
  → LLM Planner: JSON 태스크 생성 {color: red, to_zone: A, to_position: back}
    → Isaac Sim: AMR이 입고 구역(Inbound)으로 이동
    → Vision: Grounding DINO가 빨간 블록 인식 + RealSense Depth로 3D 좌표 계산
    → Planning: cuRobo가 충돌 회피 경로 생성
    → Robot: E0509이 Pick → A구역 안쪽에 Place
    → Digital Twin: 전 과정 Isaac Sim에서 실시간 미러링
```

### 출고 시나리오
```
사용자: "빨간색 물건 출고해줘"
  → LLM Planner: {color: red, to_zone: D}
    → Isaac Sim: AMR이 A구역으로 이동
    → Robot: A구역 블록 Pick → 출고 위치 Place
```

## Repository Structure

이 프로젝트는 3개의 repository로 구성되어 있습니다.

### 1. 이 Repository (LLM Planner + Digital Twin)

```
├── llm_planner/                  # LLM 기반 태스크 플래닝 + 웹 UI
│   ├── app.py                    # Flask 웹 서버 (Gradio UI, MJPEG 스트리밍)
│   ├── llm_planner.py            # Google Gemini 기반 자연어 → 로봇 태스크 변환
│   ├── main.py                   # CLI 인터페이스
│   ├── demo_scenarios.json       # 시연용 고정 시나리오 (LLM 호출 없이 즉시 응답)
│   ├── ros_interface.json        # ROS2 토픽/서비스 인터페이스 정의
│   ├── .env.example              # API 키 설정 템플릿
│   └── static/
│       └── index.html            # 웹 UI 프론트엔드
│
└── digital_twin/                 # Isaac Sim 디지털 트윈 + AMR 시뮬레이션
    ├── scripts/
    │   ├── carter_e0509_digital_twin.py   # Isaac Sim 메인 — Carter AMR + E0509 디지털 트윈
    │   ├── ros2_bridge_node.py            # ROS2 → ZMQ 브릿지 (joints/gripper/objects 전달)
    │   ├── scenarios.json                 # 구역별 시나리오 설정 (색상-구역 매핑, 위치 오프셋)
    │   ├── convert_urdf_to_usd.py         # URDF → USD 변환 유틸리티
    │   └── check_amr_height.py            # AMR 높이 검증 유틸리티
    └── assets/
        └── factory_layout.usd             # 공장 레이아웃 3D 모델
```

### 2. [e0509_gripper_description](https://github.com/AIXRobotics1-5/e0509_gripper_description)

Doosan E0509 + RH-P12-RN-A 그리퍼 통합 ROS2 패키지

| 스크립트 | 역할 |
|---------|------|
| `curobo_planner_node.py` | cuRobo GPU 가속 모션 플래닝 — 목표 pose 수신 → 충돌 회피 경로 생성 → MoveSplineJoint 실행 |
| `object_tracking_node.py` | Grounding DINO 제로샷 물체 인식 + RealSense Depth 3D 좌표 계산 → cuRobo에 목표 전송 |
| `marker_tracking_node.py` | ArUco 마커 기반 물체 추적 (Grounding DINO 대안) |
| `gripper_service_node.py` | RH-P12-RN-A 그리퍼 Modbus RTU 제어 (open/close) |
| `digital_twin_bridge.py` | 파일 기반 Joint State 브릿지 (Rviz/Gazebo 동기화용) |
| `curobo_vision.launch.py` | cuRobo + Grounding DINO 통합 launch (JIT 30초 대기 후 비전 시작) |
| `bringup.launch.py` | 로봇 bringup (실제 로봇 연결) |

### 3. [doosan-robot2](https://github.com/AIXRobotics1-5/doosan-robot2)

Doosan Robotics ROS2 드라이버 fork — FlangeSerial 서비스 4개 추가 (`flange_serial_open`, `close`, `write`, `read`)
그리퍼(RH-P12-RN-A)가 Tool Flange의 RS-485 시리얼을 통해 Modbus RTU로 통신하기 위해 필요합니다.

## Script Details

### llm_planner/app.py
Flask 기반 웹 서버로, 사용자의 자연어 명령을 받아 LLM Planner를 호출하고 결과를 ROS2 토픽으로 발행합니다.
- `/api/chat` — 자연어 명령 처리 엔드포인트
- `/stream/curobo` — cuRobo 비전 카메라 MJPEG 스트림
- `/stream/isaacsim` — Isaac Sim 뷰포트 MJPEG 스트림
- ROS2 토픽: `/dsr01/curobo/auto_task` (로봇 태스크), `/digital_twin/command` (Isaac Sim 명령)

### llm_planner/llm_planner.py
Google Gemini 2.5 Flash를 사용하여 자연어 명령을 구조화된 JSON 태스크로 변환합니다.
- 색상별 구역 자동 분류 (빨강→A, 초록→B, 노랑→C)
- 세트 주문 처리 (a세트: 빨강1+초록1 등)
- 모호한 명령 시 사용자에게 확인 질문
- 태스크 우선순위 정렬 (빨강 → 노랑 → 초록)

### digital_twin/scripts/carter_e0509_digital_twin.py
Isaac Sim 기반 디지털 트윈 메인 스크립트입니다.
- Carter AMR + E0509 로봇팔 시뮬레이션
- 명령 기반 워크플로우: `inbound` → `work A/B/C` → `outbound` → `home`
- ZMQ로 수신한 실제 로봇의 joint state를 시뮬레이션에 실시간 반영
- 비전으로 감지된 물체를 시뮬레이션에 자동 스폰

### digital_twin/scripts/ros2_bridge_node.py
ROS2와 Isaac Sim 사이의 ZMQ 브릿지입니다.
- `/dsr01/joint_states` → joints (로봇팔 동기화)
- `/dsr01/gripper/stroke` → gripper (그리퍼 동기화)
- `/dsr01/curobo/obstacles` → obstacles (감지된 물체 전달)
- `/digital_twin/command` → command (AMR 이동 명령)

## Communication Structure

```
┌──────────────┐     ROS2 Topics      ┌──────────────────┐    ZMQ (tcp:5557)    ┌──────────────┐
│   LLM Web    │ ──────────────────▶  │  ROS2 Bridge     │ ──────────────────▶  │  Isaac Sim   │
│   (app.py)   │  /auto_task          │  (ros2_bridge_   │  joints, gripper,    │  Digital     │
│              │  /digital_twin/cmd   │   node.py)       │  obstacles, command  │  Twin        │
└──────────────┘                      └────────┬─────────┘                      └──────────────┘
                                               │ subscribes
                                               ▼
                                      ┌──────────────────┐
                                      │  e0509_gripper   │
                                      │  _description    │
                                      │                  │
                                      │ • curobo_planner │ ◀── /curobo/target_pose
                                      │ • object_tracking│ ──▶ /curobo/obstacles
                                      │ • gripper_service│
                                      └────────┬─────────┘
                                               │
                                               ▼
                                      ┌──────────────────┐
                                      │  Doosan E0509    │
                                      │  + RH-P12-RN-A   │
                                      │  (Physical Robot) │
                                      └──────────────────┘
```

## Startup Sequence

시스템은 5개 터미널에서 순서대로 실행합니다.

### Terminal 1: Robot Bringup (반드시 첫 번째)
```bash
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
ros2 launch e0509_gripper_description bringup.launch.py mode:=real host:=<ROBOT_IP>
# → [dsr_controller2] init_node 뜰 때까지 대기
```

### Terminal 2: cuRobo + Vision (반드시 두 번째)
```bash
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
ros2 launch e0509_gripper_description curobo_vision.launch.py
# → cuRobo Planner Ready! 뜰 때까지 대기 (~30초)
```

### Terminal 3: ZMQ Bridge (순서 무관)
```bash
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
python3 digital_twin/scripts/ros2_bridge_node.py
# → Bridge started → tcp://*:5557 확인
```

### Terminal 4: Isaac Sim Digital Twin (순서 무관)
```bash
source ~/isaacsim_env/bin/activate
python3 digital_twin/scripts/carter_e0509_digital_twin.py
# → Ready — IDLE at home 확인
```

### Terminal 5: LLM Web App (순서 무관)
```bash
source /opt/ros/humble/setup.bash
source ~/doosan_ws/install/setup.bash
cd llm_planner
python3 app.py
# → 브라우저에서 http://localhost:7860 접속
```

## Hardware

| 장비 | 모델 | 용도 |
|------|------|------|
| 협동 로봇 | Doosan E0509 (6축) | Pick & Place 작업 |
| 그리퍼 | ROBOTIS RH-P12-RN-A | 물체 파지 (Modbus RTU) |
| 카메라 | Intel RealSense D455F | RGB + Depth 물체 인식 |
| GPU | NVIDIA (CUDA 12.8) | cuRobo 모션 플래닝 + Grounding DINO 추론 |

## Dependencies

- ROS2 Humble
- Isaac Sim 4.x
- NVIDIA cuRobo
- Grounding DINO
- Google Generative AI SDK (`google-genai`)
- Flask, PyZMQ, OpenCV, pyrealsense2

## License

This project is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
