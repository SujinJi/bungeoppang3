# 붕어빵 로봇 (bungeoppang3)

두산 협동로봇 **E0509** + **RH-P12-RN-A** 그리퍼로 붕어빵을 굽는 ROS 2 패키지입니다.
손님을 카메라로 인식하고, 음성으로 주문을 받아, 반죽 → 속재료 → 닫기 → 굽기 → 뒤집기 → 펼치기까지 자동으로 진행합니다.

- ROS 2 Jazzy · Python
- 두산 에뮬레이터(시뮬레이션)와 실물 로봇 모두 지원

## 공정 순서

| 단계 | 이름 | 내용 |
|---|---|---|
| 0 | 홈 | 홈 자세로 이동 |
| 1 | 1차 반죽 | 보조 테이블의 비커를 잡아 틀마다 '먼 판' 위에서 25° 기울여 2초 붓기 |
| 2 | 속재료 | 소스통(팥/슈크림)을 잡아 틀 위에서 뒤집고 칸 안쪽에 짜기 |
| 3 | 2차 반죽 | 1단계와 같음 (1초) |
| 4 | 뚜껑 닫기 | 가까운 손잡이를 옆에서 잡아 경첩 축으로 180° 넘겨 덮기 |
| 5 | 굽기 대기 | 10초 |
| 6 | 돌림판 + 뒤집기 | 경첩 봉을 잡아 틀을 90° 돌린 뒤, 겹친 손잡이를 들어 손목 180° 회전 |
| 7 | 다시 뒤집기 | 잡은 채 다시 들어 손목을 반대로 180° → 원래 면이 위로 오게 내려놓기 |
| 8 | 펼치기 + 원위치 | 겹친 손잡이를 놓고 빠진 뒤 위 손잡이만 위에서 잡아 105°까지 펼치고 놓기 → 손잡이 끝 밖으로 빠지기 → 돌림판 원위치 |

모든 이동 전에 안전 영역, 작업반경(850mm), 관절 범위, 들어 올리는 무게(≤ 5kg)를 검사합니다.

## 노드

| 실행 이름 | 파일 | 역할 |
|---|---|---|
| `bake_sequence` | `bake_sequence_node.py` | 전체 굽기 공정 |
| `orchestrator` | `orchestrator_node.py` | 총괄 상태 머신 (손님 인식 → 주문 → 굽기 시작) |
| `voice_node` | `voice_node.py` | 한국어 음성인식 (faster-whisper, 오프라인) |
| `vision_node` | `vision_node.py` | 웹캠 + YOLO 사람 인식 |
| `gripper_viz` | `gripper_viz.py` | RViz에 그리퍼·작업대·물체 표시 |
| `flip_mold` | `flip_mold_node.py` | 틀 뒤집기만 단독 실행 |

### 주요 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/bungeoppang/person` | `std_msgs/Bool` | 새 손님 등장 |
| `/bungeoppang/person_count` | `std_msgs/Int32` | 현재 보이는 사람 수 |
| `/bungeoppang/voice_text` | `std_msgs/String` | 인식된 문장 |
| `/bungeoppang/order` | `std_msgs/String` | 주문 (예: `팥x2`, `슈크림x1`) |
| `/bungeoppang/start_bake` | — | 굽기 시작 요청 |
| `/bungeoppang/bake_done` | `std_msgs/Bool` | 굽기 완료 |

## 설치

필요한 것:
- ROS 2 Jazzy
- [doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2) (`dsr_msgs2`, 에뮬레이터)
- RH-P12-RN-A 그리퍼 description 패키지 (`rh_p12_rn_a_description`)

```bash
cd ~/ros2_ws/src
git clone https://github.com/SujinJi/bungeoppang3.git bungeoppang2
cd ~/ros2_ws
colcon build --packages-select bungeoppang2
source install/setup.bash
```

음성인식·사람 인식을 쓰려면 (한 번만):

```bash
pip install faster-whisper sounddevice numpy ultralytics opencv-python --break-system-packages
sudo apt install -y libportaudio2
```

## 실행

### 시뮬레이션 (터미널 하나)

```bash
ros2 launch bungeoppang2 sim.launch.py
```
에뮬레이터 + RViz를 띄우고, 20초 뒤 전체 공정이 자동으로 시작됩니다.

옵션:
- `bake:=false` — 에뮬레이터 + RViz만 실행 (공정은 나중에 `ros2 run`으로)
- `f1:=슈크림 f2:=팥` — 틀 1·2의 속재료 (기본: 팥, 슈크림)
- `bake_delay:=25.0` — 에뮬레이터 준비 대기 시간(초)

### 손님 응대 데모

```bash
ros2 launch bungeoppang2 demo.launch.py          # 음성인식 끄기: voice:=false
```
다른 터미널에서 손님 등장 신호를 보내면 인사 → "팥 주세요" → 끄덕 → 공정이 시작됩니다.

```bash
ros2 topic pub --once /bungeoppang/person std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /bungeoppang/order std_msgs/msg/String "{data: 팥x1}"   # 음성 없이 주문
```

창을 4개로 나눠 띄우려면 `scripts/bungeo_split.sh`를 실행하세요.

### 공정만 실행

```bash
ros2 run bungeoppang2 bake_sequence 팥 슈크림         # 1회 실행 (틀별 속재료)
ros2 run bungeoppang2 bake_sequence 팥 --start=4      # 4단계(뚜껑 닫기)부터
ros2 run bungeoppang2 bake_sequence --real            # 실물 로봇 (단계마다 Enter로 확인)
ros2 run bungeoppang2 bake_sequence --serve           # 주문 대기 모드
```

## 값 조정

| 파일 | 내용 |
|---|---|
| `bungeoppang2_yammy/positions.py` | 위치·높이·각도·속도·무게·시간 (`★` 표시가 조정할 값) |
| `bungeoppang2_yammy/grip_params.py` | 그리퍼 폭·잡는 힘 |

`[도달 실패]` 오류가 나면 로그에 나온 목표/실제 좌표를 보고 `positions.py`에서 해당 위치를 조정하세요.
예: 펼치기 180°는 로봇이 도달할 수 없어서 `OPEN_RELEASE_DEG = 105.0`까지만 펼치고 놓습니다 (90°를 넘으면 중력으로 마저 눕혀짐). 실행 중에는 팔이 닿는 각도를 미리 확인해서 그보다 덜 펼 수 있고, `OPEN_RELEASE_MIN`(95°)보다 덜 닿으면 멈춥니다.

## 에뮬레이터 실행 영상

[Rviz2 시연 영상](https://youtu.be/tOi8jhYWsEc)


## 라이선스

MIT
