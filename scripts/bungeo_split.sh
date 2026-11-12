#!/bin/bash
# 붕어빵 데모: 이전 실행 정리 → 터미널 창 4개 (에뮬+RViz / 공정 / 총괄 / 음성)
pkill -f ros2_control_node; pkill -f robot_state_publisher; pkill -f spawner
pkill -f rviz2; pkill -f bungeoppang2; pkill -f run_emulator
docker stop dsr01_emulator >/dev/null 2>&1
ros2 daemon stop >/dev/null 2>&1
sleep 2
S="source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash"
open_win() {
  gnome-terminal --window --title="$1" -- bash -c "$S; echo '=== $1 ==='; $2; exec bash" \
    || echo "❌ 창을 열 수 없어요: $1"
}
open_win "1 에뮬+RViz" "ros2 launch bungeoppang2 sim.launch.py bake:=false"
echo "로봇 뜰 때까지 25초 기다려요..."
sleep 25
open_win "2 공정"  "ros2 run bungeoppang2 bake_sequence --serve"
sleep 1
open_win "3 총괄"  "ros2 run bungeoppang2 orchestrator"
sleep 1
open_win "4 음성"  "ros2 run bungeoppang2 voice_node"
echo "✅ 4개 창을 열었어요. '2 공정' 창에 '로봇 연결 확인 완료'가 뜨면 말하세요."
