"""★★ 위치 · 물체 길이 전용 파일 ★★

작업대, 안전 영역, 물체 치수, 티칭 좌표, 속도를 모아 뒀어요.
그리퍼 힘·폭 값은 grip_params.py에 있어요. 단위는 mm와 도(deg).

[좌표 약속 — RViz 베이스 모델 기준으로 정정]
  틀(작업대 긴 방향) = 로봇 ±X,  짧은 방향 = 로봇 ±Y
  베이스의 커넥터(버튼처럼 튀어나온 부분)는 짧은 방향(좁은 7cm 쪽)을 향해요.
  틀이 +X 쪽인지 -X 쪽인지는 아래 MOLD_SIGN 하나로 정해요.
  → 실물에서 티치펜던트 +X 조그를 눌러 틀 쪽으로 가면 +1, 반대로 가면 -1.
  물체들이 좌우(짧은 방향) 반대쪽에 있으면 MOLD_SHORT 등의 부호를 뒤집으세요.
"""
import math

MOLD_SIGN = -1.0         # ★ 틀(작업대 긴 쪽)이 로봇 +X면 +1, -X면 -1 (조그로 확인!)
SHORT_SIGN = +1.0        # ★ 짧은 방향 부호 (도면: 커넥터 쪽 좁은 7cm 면이 위). 거울로 보이면 부호 반대
#   (베이스 180° 회전 = MOLD_SIGN과 SHORT_SIGN을 함께 뒤집기)


def pour_az():
    """주전자를 기울일 때 그리퍼 축이 넘어가는 방향(틀 반대쪽) — 그래야 주둥이가 틀 쪽으로 내려감"""
    return 0.0 if MOLD_SIGN < 0 else 180.0


def top_pose(x, y, z_tcp):
    """위에서 수직으로 내려 잡는 자세. z_tcp = 손가락 끝(잡는 점) 높이.
    A-C 차이가 손목 회전(KETTLE_YAW)이라, 기울일 때 B만 바꾸면 매끄럽게 기울어요."""
    a = pour_az()
    return [x, y, z_tcp + TOOL_LENGTH, a, 180.0, a - KETTLE_YAW]


def side_grip(long_, short, z):
    """옆에서 수평으로 잡되 손가락이 '좌우'로 닫히는 자세 (세워진 통을 몸통 중간에서 잡기).
    pose_to는 손가락이 위아래로 닫혀서 세워진 통에 쓰면 통이 눕혀져요."""
    q = pose_to(long_, short, z)
    q[5] = 0.0
    return q


def radial_grip(long_, short, z):
    """세워진 원기둥(소스통)을 옆면에서 잡는 자세.
    로봇 1축(바닥)을 돌려 통 쪽을 바라보고, 베이스에서 통으로 뻗는 방향으로 수평 접근,
    손가락은 좌우로 닫힘. (long_, short, z) = 통 중심(잡는 점) 위치"""
    x, y = MOLD_SIGN * long_, SHORT_SIGN * short
    r = math.hypot(x, y)
    ux, uy = x / r, y / r
    a = math.degrees(math.atan2(uy, ux))
    return [x - ux * TOOL_LENGTH, y - uy * TOOL_LENGTH, z, a, 90.0, 0.0]


def outer_grip(long_, short, z):
    """세워진 통을 '바깥에서' 잡는 자세: 그리퍼가 통 너머에서 로봇 쪽을 바라보고
    손가락이 좌우로 닫힘. 로봇 가까이 둔 물체도 팔이 펴진 채 잡을 수 있어요."""
    x, y = MOLD_SIGN * long_, SHORT_SIGN * short
    r = math.hypot(x, y)
    ux, uy = x / r, y / r
    return [x + ux * TOOL_LENGTH, y + uy * TOOL_LENGTH, z, math.degrees(math.atan2(-uy, -ux)), 90.0, 0.0]


def pose_to(long_, short, z):
    """작업대 기준(틀 쪽 거리, 짧은 방향, 높이) → 로봇 posx [x,y,z,A,B,C].
    그리퍼가 틀 쪽을 수평으로 향하고 손가락이 위아래로 닫히는 자세예요.
    MOLD_SIGN을 바꾸면 모든 좌표와 자세가 자동으로 반대쪽으로 뒤집혀요."""
    a = 0.0 if MOLD_SIGN > 0 else 180.0         # 툴 z가 틀 쪽(±X)을 향하도록
    return [MOLD_SIGN * long_, SHORT_SIGN * short, z, a, 90.0, 90.0]

# ============================================================
# 1. 작업대 (도면 치수에서 계산)  — 긴 방향 = Y, 짧은 방향 = X
# ============================================================
TABLE_FWD = 901.0        # ★ 베이스 중심 → 틀 쪽 끝 (도면: 8.5 + 81.6cm. 가로 합계가 120과 6cm 차이 → 실측 확인)
TABLE_BACK = 238.0       # ★ 베이스 중심 → 반대쪽 끝 (15.3 + 8.5cm)
# 도면 세로: 7 + 21 + 26.5 = 54.5 ≈ 55cm
TABLE_SHORT_A = 375.0    # ★ +Y(넓은 쪽, 도면 아래) 끝: 26.5 + 10.5cm
TABLE_SHORT_B = 175.0    # ★ -Y(커넥터가 향하는 좁은 쪽) 끝: 7 + 10.5cm
PEDESTAL_LONG = 170.0    # ★ 로봇 받침대: 작업대 긴 방향 17cm
PEDESTAL_SHORT = 210.0   # ★ 받침대: 짧은 방향 21cm
PEDESTAL_H = 70.0        # ★ 받침대 높이 7cm (로봇 베이스가 이 위에 올라감)
HANDLE_APPROACH_BACK = 50.0  # ★ 틀 손잡이는 받침대와 가까워서 짧게 뒤에서 접근
TABLE_TOP_Z = -PEDESTAL_H   # 작업대 윗면 = 베이스보다 받침대 높이만큼 아래
# ---- 로봇 뒤쪽에 붙이는 보조 테이블 (소스통 올려두는 곳) ----
TABLE2_FWD = (-730.0, -238.0)    # ★ 틀 방향 범위 (로봇 뒤쪽, 기존 작업대 뒤 끝에 붙음) — 실측으로 교체
TABLE2_SHORT = (-245.0, 390.0)   # ★ 짧은 방향 범위 — 실측으로 교체

# ============================================================
# 2. 안전 영역  ← "틀 쪽으로 너무 가면 멈추는 문제" 방지
# ============================================================
SAFE_FWD_MAX = 600.0     # ★ 틀 방향 한계 (손끝 기준). 실제 멈춘 위치보다 10cm 안쪽으로
SAFE_RADIUS_MAX = 700.0  # ★ 베이스에서 수평 거리 한계
SAFE_RADIUS_MIN = 150.0  # ★ 베이스에 너무 붙는 자세 방지
SAFE_EDGE_MARGIN = 30.0  # ★ 작업대 가장자리 여유
TRANSIT_FREE_Z = TABLE_TOP_Z + 200.0  # ★ 이 높이 이상으로 들면 작업대 가장자리 검사를 안 함
#   (들거나 이동할 때는 테이블 밖으로 나가도 됨. 낮게 움직일 때만 가장자리 검사)
SAFE_Z_MIN = 30.0        # ★ 손끝·플랜지·손목이 로봇 베이스보다 3cm 아래로 내려가지 않게 (베이스 아래로 가면 서보 오프 위험)
SAFE_Z_MAX = 700.0

# ============================================================
# 3. 물체 길이 (실측하면 여기만 수정)
# ============================================================
TOOL_LENGTH = 160.0      # ★ 플랜지 면 → 손가락 끝 (브래킷 포함, 실측)
GRIPPER_YAW_DEG = 0.0    # ★ 그리퍼가 플랜지에 돌아가 붙은 각도 (RViz용)
GRIPPER_MOUNT_Z = 0.0    # ★ 브래킷 두께 (RViz용)

MOLD_L = 90.0            # ★ 틀 하나의 폭 9cm (짧은 방향). 틀 2개가 나란히 → 총 18cm
GRILL_FWD, GRILL_SHORT = 230.0, 180.0   # ★ 두 틀을 고정한 철판(석쇠) 23 x 18cm
MOLD_W = 77.5            # ★ 판 하나 깊이 = 펼친 길이 15.5cm의 절반
MOLD_FLOAT = 160.0       # ★ 틀(돌림판 위) 높이 = 작업대 위 16cm → 로봇 베이스(7cm)보다 9cm 위
FISH_L, FISH_W = 80.0, 50.0
FISH_INSET = 10.0        # ★ 생선 칸 네모에서 가로·세로 1cm 안쪽에 타원
FISH_DEPTH = 10.0        # ★ 타원 파인 깊이 1cm
HANDLE_L, HANDLE_W, HANDLE_T = 150.0, 50.0, 20.0   # ★ 손잡이
STRING_LEN = 60.0        # ★ 위쪽 손잡이 → 끈 고리까지
SAUCE_DIA = 63.0         # ★ 소스통 지름 6.3cm
SAUCE_H = 225.0          # ★ 소스통 높이 22.5cm (노즐 포함)
SAUCE_BASE_H = 160.0     # ★ 소스통 밑 상자 높이 (틀과 같은 높이 = 베이스보다 9cm 위)
SAUCE_BOTTOM_Z = TABLE_TOP_Z + SAUCE_BASE_H
# 주전자 손잡이는 뚜껑 위로 솟은 형태(하늘을 향함) → 그리퍼가 위에서 아래로 잡아요
# 비커(계량컵): 몸통 지름 7.5cm, 높이 8.4cm, 전체 길이 16.5cm (주둥이 ↔ 옆 손잡이)
CUP_D, CUP_H = 75.0, 84.0       # ★ 비커 몸통 지름·높이
CUP_HANDLE_REACH = 82.5         # ★ 손잡이 잡는 점 → 몸통 중심 (수평)
CUP_GRIP_H = 50.0               # ★ 손잡이 잡는 높이 (작업대에서)
CUP_APPROACH = 40.0             # ★ 손잡이에 옆에서 다가가는 거리
POUR_GAP_CUP = CUP_H + 10.0     # 주둥이가 테두리에 있어서, 몸통 바닥이 틀에 안 닿게 주둥이를 이만큼 띄움
POUR_A_DEG = 215.0              # ★ 부을 때 그리퍼 방향 (계산으로 고른 값: 팔 안 접히고 안전 영역 안)
KETTLE_SPOUT_DROP = -(CUP_H - CUP_GRIP_H)   # 잡는 점 → 주둥이 끝 높이 차 (주둥이가 34mm 위)
KETTLE_SPOUT_REACH = CUP_HANDLE_REACH + CUP_D / 2 + 12.0   # 잡는 점 → 주둥이 끝 수평 거리 (132mm)
KETTLE_YAW = 90.0           # ★ 손가락이 손잡이를 가로질러 닫히도록 돌리는 각도 (0/90 중 실물에 맞게)
#   주둥이를 틀 위에 두면 그리퍼는 이만큼 뒤에 있게 돼요 → 팔이 덜 뻗어도 됨
NOZZLE_DROP = 0.0        # ★ 소스통: 그리퍼 TCP → 입구 끝 높이 차 (실측)

# ============================================================
# 4. 붓기 · 짜기
# ============================================================
POUR_GAP = 30.0          # ★ 주전자 주둥이 ↔ 틀 윗면 간격 (3cm)
FILL_GAP = 20.0          # ★ 소스통 입구 ↔ 틀 간격 (2cm)
MOLD_TOP_Z = TABLE_TOP_Z + MOLD_FLOAT + 20.0   # ★ 틀 윗면 높이 (틀 두께 2cm 가정)

# 생선 2칸 중심 오프셋 [틀방향, 짧은방향]  ★ 실물 보며 보정
# 도면 기준: 두 칸은 '짧은 방향(X)'으로 나란히 놓임 (손잡이 축과 수직)
FISH_OFFSETS = [[0.0, 0.0]]   # 판마다 생선 칸 1개(8x5cm), 판 가운데
POUR_LINE = 45.0         # ★ 붓는 동안 이동 길이 (머리→꼬리 = 손잡이 축 방향)
POUR_SPEED = 15.0        # ★ 붓는 이동 속도 (느릴수록 두껍게)
FILL_POINT_D = 0.0       # ★ 속재료 짜는 점 (생선 중심 기준, 틀방향 +)

# ============================================================
# 5. 티칭 좌표  ※ 전부 임시값! 실물에서 티칭해서 교체
#    pose_to(틀쪽 거리, 짧은방향, 높이) → MOLD_SIGN 따라 자동 변환
# ============================================================
# 틀 구조: 판 2장이 가운데 경첩으로 연결된 조개식. 기본(원래) 상태 = 활짝 열려 두 판이
# 틀 방향(Y)으로 나란히 펼쳐짐. 가까운 판 손잡이는 로봇 쪽, 먼 판 손잡이는 반대쪽.
# 각 판에는 생선 칸이 짧은 방향(X)으로 2개씩 있음.
MOLD_FWD = 170.0              # ★ 가까운 손잡이 '가로 중앙' 잡는 점까지 틀쪽 거리 (티칭으로 교체)
MOLD_SHORT = 120.0            # ★ 손잡이의 짧은방향(X) 위치
HANDLE_STICK = 40.0           # ★ 손잡이 잡는 점 → 판 가장자리까지 (손잡이 절반)
MOLD_GRIP = pose_to(MOLD_FWD, MOLD_SHORT, TABLE_TOP_Z + MOLD_FLOAT)
# 틀 1·2의 가운데 줄 (손잡이는 철판 가운데 1개, 두 틀이 그 양옆) — 1번 = 커넥터 쪽
MOLD_SHORTS = [MOLD_SHORT - MOLD_L / 2, MOLD_SHORT + MOLD_L / 2]
MOLD_GRIPS = [pose_to(MOLD_FWD, s, TABLE_TOP_Z + MOLD_FLOAT) for s in MOLD_SHORTS]
STRING_GRIP = pose_to(MOLD_FWD, MOLD_SHORT, TABLE_TOP_Z + MOLD_FLOAT - 60.0)  # (예비)
SAUCE_PICK_REDBEAN = radial_grip(-660.0, -100.0, SAUCE_BOTTOM_Z + SAUCE_H / 2)   # ★ 팥 — 뒤 테이블 끝쪽 (로봇 쪽에서 잡음)
SAUCE_PICK_CREAM = radial_grip(-660.0, 40.0, SAUCE_BOTTOM_Z + SAUCE_H / 2)       # ★ 슈크림 — 팥 옆 14cm
REACH_TOL = 5.0          # ★ 목표 자세와 실제 도착 위치 차이 허용치 (mm)
OBJ_TRANSIT_Z = SAUCE_BOTTOM_Z + SAUCE_H + 140.0   # ★ 물체를 들고/잡으러 오갈 때 손 높이 (통 든 채 다른 통 위로 지나감)
#   (통을 든 채로도 통 바닥이 주전자·틀·다른 통 위로 지나가는 높이)
SAUCE_PICKS = {"팥": SAUCE_PICK_REDBEAN, "슈크림": SAUCE_PICK_CREAM}  # 주문 이름 → 통 위치
SAUCE_PICK = SAUCE_PICK_REDBEAN     # (이름이 안 맞을 때 쓰는 예비값)
#   통 높이 12cm → 잡는 높이 TABLE_TOP_Z + 60 = 몸통 정중앙
KETTLE_FWD, KETTLE_SHORT = -640.0, 300.0    # ★ 비커 몸통 중심 — 뒤 테이블 끝쪽, 손잡이는 로봇 쪽
CUP_BASE_H = 160.0              # ★ 비커 밑 상자 높이 (틀과 같은 높이 = 베이스보다 9cm 위)
CUP_BOTTOM_Z = TABLE_TOP_Z + CUP_BASE_H
KETTLE_HANDLE_Z = TABLE_TOP_Z + CUP_BASE_H + CUP_GRIP_H   # 비커 손잡이 = 상자 + 손잡이 높이 (자동)


def _cup_pick():
    """비커: 손잡이가 로봇 쪽 옆면, 주둥이가 바깥쪽. 로봇 쪽에서 손잡이를 옆으로(수평) 잡는 자세
    (물체를 베이스보다 높이 올렸기 때문에 로봇 쪽에서 잡아도 팔이 접히지 않고, 너무 펴지지도 않음)"""
    cx, cy = MOLD_SIGN * KETTLE_FWD, SHORT_SIGN * KETTLE_SHORT
    r = math.hypot(cx, cy)
    gx, gy = cx - cx / r * CUP_HANDLE_REACH, cy - cy / r * CUP_HANDLE_REACH
    return radial_grip(gx * MOLD_SIGN, gy * SHORT_SIGN, KETTLE_HANDLE_Z)


KETTLE_PICK = _cup_pick()

APPROACH_BACK = 80.0     # ★ 잡기 전 뒤(베이스 쪽)에서 대기하는 거리
LIFT_HEIGHT = 300.0      # ★ 뒤집기 전 드는 높이 (+30cm)
FLIP_ANGLE = 180.0       # ★ 뒤집는 각도 (J6 = 손잡이 축)
RETREAT_UP = 100.0       # ★ 놓은 뒤 위로 빠지는 높이
RETREAT_SIDE = 50.0      # ★ 뒤집은 뒤 손잡이에서 옆(짧은 방향 Y)으로 떨어지는 거리 (5cm)

# ---- 제스처 (인사·대답) ----
BOW_J = [0.0, 25.0, 100.0, 0.0, 40.0, 0.0]  # ★ ㄱ자 인사 자세 (posj, 에뮬레이터로 보며 조정)
GESTURE_AMP = 60.0       # ★ 도리도리 좌우 폭 (mm)
NOD_UP = 40.0            # ★ 끄덕: 시작 높이에서 위로 (mm)
NOD_DOWN = 15.0          # ★ 끄덕: 시작 높이에서 아래로 (mm) — 너무 내려가면 줄이기
GESTURE_REPEAT = 2       # ★ 흔드는 횟수
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]   # ★ 시작/끝 자세 (posj)

# ============================================================
# 5-1. 공정 동작 값
# ============================================================
POUR_TILT_DEG = 25.0     # ★ 주전자 기울이는 각도 (25도)
POUR_HOLD1 = 2.0         # ★ 1차 반죽: 기울인 채 유지 (2초)
POUR_HOLD2 = 1.0         # ★ 2차 반죽: 유지 (1초)
POUR_RATIO = 2.0 / 3.0   # ★ 붓는 점: 틀 깊이의 2/3 지점 (손잡이에서 먼 쪽)
POUR_HOP = 80.0          # ★ 칸 사이 이동할 때 띄우는 높이

SAUCE_LIFT = 150.0       # ★ 소스통 수직으로 드는 높이
FLIP_CLEAR = 150.0       # ★ 입구 뒤집을 때 여유 높이 (통 절반 길이보다 크게)
SAUCE_GRIP_TO_MOUTH = 112.5  # ★ 잡는 점(통 중간) → 노즐 끝 (통 높이 절반, 실측으로 보정)
DISPENSE_INSET = 10.0    # ★ 틀 가장자리에서 1cm 안쪽까지만 짜기
DISPENSE_DIR = +1.0      # ★ '아래서 위로'의 시작 쪽: +1 = +Y(넓은 쪽)에서 시작. 반대면 -1
DISPENSE_HOP = 50.0      # ★ 짠 뒤 띄우는 높이

# ---- 뚜껑 (경첩 축 = 짧은 방향 X, 뚜껑은 틀 방향으로 열림) ----
# 경첩 = 두 판 사이 가운데. 위치는 손잡이에서 자동 계산:
#   경첩 = 잡는 점 + 그리퍼 길이 + 손잡이 절반 + 판 깊이
HINGE_FWD = MOLD_FWD + TOOL_LENGTH + HANDLE_STICK + MOLD_W   # (= 손잡이 티칭을 따라감)
HINGE_Z = TABLE_TOP_Z + MOLD_FLOAT + 20.0   # ★ 경첩 높이
LID_R = HANDLE_STICK + MOLD_W   # 경첩 → 반대쪽 손잡이 잡는 점 (판 깊이 + 손잡이 절반)
#   이 값이면 닫았을 때 위 손잡이가 아래 손잡이 위에 정확히 포개져요 (총 두께 4cm)
LID_OPEN_SIGN = +1.0     # 먼 쪽으로 열림 (조개식 구조상 고정)
# ⚠ 꼭 확인: 활짝 연 먼 손잡이 위치(HINGE_FWD + LID_R)가 틀 방향 한계 SAFE_FWD_MAX(600)
#   안이어야 해요. 지금 임시값: 경첩 460 + 반지름 130 = 590 < 600 ✓ (MOLD_FWD를 키우면 초과 주의)

LID_STEP_DEG = 15.0      # ★ 호 경유점 간격
STRING_SLACK = 20.0      # ★ 끈 당길 때 반지름 여유
STRING_RELEASE_DEG = 120.0  # ★ 끈을 이 각도까지만 당기고 놓기 (그 뒤는 중력이 뚜껑을 마저 넘김)
#   끈 호는 반지름이 커서 180°까지 따라가면 틀 방향 한계를 넘어요. 90~130° 사이에서 조절
OPEN_RELEASE_DEG = 145.0  # ★ 뒤집은 뒤 펼치기: 이 각도까지만 펼치고 놓기 (나머지는 중력으로 눕혀짐)
#   180°(손끝 y≈270, 플랜지 [-448, 430, 90])는 로봇이 'NOT REACHABLE'(특이점)이라 못 감. 150~170° 사이에서 조절

# ============================================================
# 6. 속도 · 시간
# ============================================================
SPEED_SCALE_REAL = 0.2   # ★ 실물 속도 배율
VEL_J, ACC_J = 40.0, 40.0
VEL_L, ACC_L = 100.0, 100.0
VEL_SLOW_L = 30.0
VEL_FLIP = 30.0
J6_LIMIT = 355.0

# ================= 두산 E0509 사양 — 반드시 지킴 (공식 매뉴얼) =================
SPEC_REACH = 900.0                               # 최대 작업 반경 (베이스 중심에서, mm)
SPEC_PAYLOAD_KG = 5.0                            # 가반하중 (kg)
SPEC_TCP_VEL = 1000.0                            # 최대 TCP 속도 (mm/s)
SPEC_JOINT_RANGE = [360.0, 360.0, 155.0, 360.0, 155.0, 360.0]   # J1~J6 가동 범위 ±(°)
SPEC_JOINT_VEL = [120.0, 120.0, 150.0, 225.0, 225.0, 225.0]     # J1~J6 최대 속도 (°/s)
JOINT_MARGIN = 5.0         # ★ 관절 한계 여유 (°): J3·J5는 ±150°까지만 사용
REACH_MARGIN = 50.0        # ★ 반경 여유 (mm): 850mm까지만 사용

# 들어 올리는 물체 무게 (kg) — ★ 실측해서 바꾸세요. 그리퍼 무게 + 물체 무게 ≤ 5kg
GRIPPER_KG = 0.5           # ★ RH-P12-RN 그리퍼 + 브래킷
OBJ_KG = {
    "kettle": 0.6,         # ★ 비커 + 반죽
    "sauce": 0.4,          # ★ 소스통 1개 (가득 찼을 때)
    "mold": 3.0,           # ★ 뒤집을 때 드는 틀 전체(판 2장 + 손잡이, 철판 포함이면 그만큼) — 꼭 실측!
    "lid": 1.0,            # ★ 뚜껑 판 1장 (닫기·열기 때 들림)
    "turn": 0.0,           # 돌림판은 밀어서 돌림 (들지 않음)
}


def joint_problems(q):
    """관절값 q(°)가 사양 범위(여유 포함)를 넘으면 문제 목록"""
    out = []
    for i, (v, lim) in enumerate(zip(q, SPEC_JOINT_RANGE)):
        if abs(v) > lim - JOINT_MARGIN:
            out.append(f"J{i+1}={v:.0f}° (한계 ±{lim:.0f}°)")
    return out


def check_payload(kind, label=""):
    kg = GRIPPER_KG + OBJ_KG.get(kind.split(":")[0], 0.0)
    if kg > SPEC_PAYLOAD_KG:
        raise ValueError(f"[안전 정지] {label}: 무게 {kg:.1f}kg > 가반하중 {SPEC_PAYLOAD_KG:.0f}kg")
    return kg


def _spec_selfcheck():
    """설정값이 사양을 넘으면 시작 자체를 막음"""
    bad = []
    if VEL_J > min(SPEC_JOINT_VEL):
        bad.append(f"VEL_J {VEL_J}°/s > 관절 최대 {min(SPEC_JOINT_VEL):.0f}°/s")
    for nm, v in (("VEL_L", VEL_L), ("VEL_SLOW_L", VEL_SLOW_L)):
        if v > SPEC_TCP_VEL:
            bad.append(f"{nm} {v}mm/s > TCP 최대 {SPEC_TCP_VEL:.0f}mm/s")
    if VEL_FLIP > SPEC_JOINT_VEL[5]:
        bad.append(f"VEL_FLIP {VEL_FLIP}°/s > J6 최대 {SPEC_JOINT_VEL[5]:.0f}°/s")
    if J6_LIMIT > SPEC_JOINT_RANGE[5]:
        bad.append(f"J6_LIMIT {J6_LIMIT}° > ±{SPEC_JOINT_RANGE[5]:.0f}°")
    for k in OBJ_KG:
        if GRIPPER_KG + OBJ_KG[k] > SPEC_PAYLOAD_KG:
            bad.append(f"{k} 무게 {GRIPPER_KG + OBJ_KG[k]:.1f}kg > {SPEC_PAYLOAD_KG:.0f}kg")
    if bad:
        raise ValueError("[사양 위반] " + " / ".join(bad))
BAKE_SEC_SIDE1 = 10.0    # ★ 뚜껑 닫고 기다리는 시간 (10초, 실물은 굽는 시간으로)
BAKE_SEC_SIDE2 = 10.0    # ★ 뒤집은 채 잡고 기다리는 시간 (10초)
BAKE_SEC_SIDE3 = 0.0     # ★ 다시 뒤집어 원위치한 뒤 기다리는 시간 (0 = 바로 펼치기)


# ============================================================
# 좌표 변환 · 안전 검사 (수정하지 않아도 돼요)
# ============================================================
def fwd_of(x, y):
    """로봇 좌표 → 틀 방향 거리 (틀 방향 = X, MOLD_SIGN 반영)"""
    return x * MOLD_SIGN


def tool_dir(pose):
    """툴 z(그리퍼가 향하는 방향) 단위벡터"""
    a, b = math.radians(pose[3]), math.radians(pose[4])
    return (math.cos(a) * math.sin(b), math.sin(a) * math.sin(b), math.cos(b))


def tool_tip(pose):
    zx, zy, zz = tool_dir(pose)
    return pose[0] + TOOL_LENGTH * zx, pose[1] + TOOL_LENGTH * zy, pose[2] + TOOL_LENGTH * zz


SHOULDER_Z = 160.0       # E0509 어깨(2축) 높이 (대략)
WRIST_BACK = 124.0       # 플랜지 → 손목 중심 (대략)
FOLD_MIN = 230.0         # ★ 어깨~손목 거리가 이보다 짧으면 3축이 한계(155°) 근처로 접힘


def arm_fold(pose):
    """어깨~손목 중심 거리(mm). 작을수록 팔이 많이 접혀 3축 한계에 걸리기 쉬움"""
    u = tool_dir(pose)
    w = [pose[i] - u[i] * WRIST_BACK for i in range(3)]
    return math.dist(w, [0.0, 0.0, SHOULDER_Z])


WRIST_REACH_MAX = 650.0  # ★ 어깨~손목 최대 거리 (에뮬레이터 약 700mm에서 멈춤 → 여유 두고 680)
FOLD_PATH_MIN = 250.0    # ★ 직선 이동 중 어깨~손목 최소 거리 (이보다 짧으면 3축 한계에 걸림)


def check_path(a, b, label="", n=30):
    """a → b 직선 이동 중간 지점들까지 팔 접힘 검사 (끝점만 괜찮아도 중간에 걸릴 수 있어요)"""
    worst = None
    for k in range(n + 1):
        p = [a[i] + (b[i] - a[i]) * k / n for i in range(3)] + list(b[3:6])
        f = arm_fold(p)
        if worst is None or f < worst[0]:
            worst = (f, p)
    far = max(arm_fold([a[i] + (b[i] - a[i]) * k / n for i in range(3)] + list(b[3:6])) for k in range(n + 1))
    if far > WRIST_REACH_MAX:
        raise ValueError(f"[안전 정지] {label}: 이동 중 팔이 너무 펴짐 (어깨~손목 {far:.0f}mm > {WRIST_REACH_MAX:.0f})")
    if worst[0] < FOLD_PATH_MIN:
        w = [round(v) for v in worst[1][:3]]
        raise ValueError(f"[안전 정지] {label}: 이동 중 팔이 너무 접힘 (지점 {w}, "
                         f"어깨~손목 {worst[0]:.0f}mm < {FOLD_PATH_MIN:.0f}) → 물체를 로봇에서 더 멀리 두세요")


def check_pose(pose, label=""):
    """목표 자세가 안전 영역 밖이면 ValueError. 플랜지와 손끝 둘 다 검사."""
    problems = []
    m = SAFE_EDGE_MARGIN
    for name, (x, y, z) in (("플랜지", tuple(pose[:3])), ("손끝", tool_tip(pose))):
        fwd = fwd_of(x, y)
        r = math.hypot(x, y)
        reach = max(math.dist((x, y, z), (0.0, 0.0, 0.0)), math.dist((x, y, z), (0.0, 0.0, SHOULDER_Z)))
        if reach > SPEC_REACH - REACH_MARGIN:
            problems.append(f"{name} 작업 반경 {reach:.0f}mm > {SPEC_REACH - REACH_MARGIN:.0f} (사양 최대 {SPEC_REACH:.0f})")
        if fwd > SAFE_FWD_MAX:
            problems.append(f"{name} 틀방향 {fwd:.0f} > 한계 {SAFE_FWD_MAX:.0f}")
        if r > SAFE_RADIUS_MAX:
            problems.append(f"{name} 거리 {r:.0f} > 최대 {SAFE_RADIUS_MAX:.0f}")
        if r < SAFE_RADIUS_MIN:
            problems.append(f"{name} 거리 {r:.0f} < 최소 {SAFE_RADIUS_MIN:.0f}")
        if z < TRANSIT_FREE_Z and name == "손끝":   # 손끝만 작업대 위면 됨 (팔·손목은 밖으로 나가도 OK), 높이 들면 면제
            s_ = y * SHORT_SIGN
            on1 = (-TABLE_BACK + m <= fwd <= TABLE_FWD - m) and (-TABLE_SHORT_B + m <= s_ <= TABLE_SHORT_A - m)
            on2 = (TABLE2_FWD[0] + m <= fwd <= TABLE2_FWD[1] + m) and (TABLE2_SHORT[0] + m <= s_ <= TABLE2_SHORT[1] - m)
            if not (on1 or on2):
                problems.append(f"{name} (틀방향 {fwd:.0f}, 짧은방향 {s_:.0f}) 작업대 밖 (낮은 높이)")
        if z < SAFE_EDGE_MARGIN and abs(x) < PEDESTAL_LONG / 2 + m and abs(y) < PEDESTAL_SHORT / 2 + m:
            problems.append(f"{name} 받침대와 충돌 (x={x:.0f}, y={y:.0f}, z={z:.0f})")
        if not (SAFE_Z_MIN <= z <= SAFE_Z_MAX):
            problems.append(f"{name} z={z:.0f} 높이 범위({SAFE_Z_MIN:.0f}~{SAFE_Z_MAX:.0f}) 밖")
    u_ = tool_dir(pose)
    wz = pose[2] - u_[2] * WRIST_BACK
    if wz < SAFE_Z_MIN:
        problems.append(f"손목 z={wz:.0f} < 베이스 위 {SAFE_Z_MIN:.0f}mm (베이스보다 낮아짐)")
    if arm_fold(pose) > WRIST_REACH_MAX:
        problems.append(f"팔이 너무 펴짐 (어깨~손목 {arm_fold(pose):.0f}mm > {WRIST_REACH_MAX:.0f}, 닿지 않음)")
    if arm_fold(pose) < FOLD_MIN:
        problems.append(f"팔이 너무 접힘 (어깨~손목 {arm_fold(pose):.0f}mm < {FOLD_MIN:.0f}, 3축 한계 위험)")
    if problems:
        raise ValueError(f"[안전 정지] {label}: " + " / ".join(problems))


def _seg_rect_gap(px, py, rect):
    """점(로봇 xy) ↔ 사각형(틀쪽 거리, 짧은방향 범위) 거리"""
    f, s = px * MOLD_SIGN, py * SHORT_SIGN
    (f0, f1), (s0, s1) = rect
    dx = max(f0 - f, 0.0, f - f1)
    dy = max(s0 - s, 0.0, s - s1)
    return math.hypot(dx, dy)


def _obstacles():
    """붓는 동안 피해야 할 것: 틀 손잡이 2개(가까운/먼), 소스통 2개"""
    grip_f = MOLD_FWD + TOOL_LENGTH                 # 가까운 손잡이 잡는 점
    edge = grip_f + HANDLE_STICK                    # 가까운 판 가장자리
    far = edge + 2 * MOLD_W                         # 먼 판 가장자리
    rects = []
    for ms in HANDLE_SHORTS:                        # 앞·뒤 손잡이
        hs = (ms - HANDLE_W / 2, ms + HANDLE_W / 2)
        rects += [((edge - HANDLE_L, edge), hs), ((far, far + HANDLE_L), hs)]
    for b in SAUCE_PICKS.values():
        t = tool_tip(b)
        f, s = t[0] * MOLD_SIGN, t[1] * SHORT_SIGN
        r = SAUCE_DIA / 2
        rects.append(((f - r, f + r), (s - r, s + r)))
    return rects


POUR_CLEAR = 15.0        # ★ 비커·그리퍼와 손잡이·소스통 사이 최소 여유 (위에서 본 거리)
GRIPPER_HALF_W = 45.0    # ★ 그리퍼 몸통 반폭


def _pour_poses(S, A):
    """주둥이 목표 S, 그리퍼 방향 A → (수평 자세 F, 기울인 자세 T, 이동 높이 H)"""
    u = (math.cos(math.radians(A)), math.sin(math.radians(A)))
    L = KETTLE_SPOUT_REACH + TOOL_LENGTH
    D = KETTLE_SPOUT_DROP
    ph = math.radians(POUR_TILT_DEG)
    F = [S[0] - u[0] * L, S[1] - u[1] * L, S[2] + D, A, 90.0, 0.0]
    hor = -L * math.cos(ph) + D * math.sin(ph)
    vz = L * math.sin(ph) + D * math.cos(ph)
    T = [S[0] + u[0] * hor, S[1] + u[1] * hor, S[2] + vz, A, 90.0 + POUR_TILT_DEG, 0.0]
    H = list(F); H[2] = OBJ_TRANSIT_Z
    return F, T, H


def _pour_clear(S, A, rects):
    """위에서 봤을 때 비커 몸통·그리퍼 몸통이 손잡이·소스통과 안 겹치는지 (최소 거리 반환)"""
    u = (math.cos(math.radians(A)), math.sin(math.radians(A)))
    c = (S[0] - u[0] * (CUP_D / 2 + 12.0), S[1] - u[1] * (CUP_D / 2 + 12.0))
    gaps = [_seg_rect_gap(c[0], c[1], r) - CUP_D / 2 for r in rects]
    tcp = KETTLE_SPOUT_REACH
    for d in range(0, int(TOOL_LENGTH + WRIST_BACK) + 1, 20):      # 손가락 끝 → 손목
        p = (S[0] - u[0] * (tcp + d), S[1] - u[1] * (tcp + d))
        gaps += [_seg_rect_gap(p[0], p[1], r) - GRIPPER_HALF_W for r in rects]
    return min(gaps)


def plan_pour(grip):
    """주둥이를 이 틀의 노란 생선 칸 안에 두고, 손잡이·소스통을 피하면서
    팔이 가장 편한 그리퍼 방향을 고름 → (주둥이 목표 S, 방향 A)"""
    md, t0 = tool_dir(grip), tool_tip(grip)
    k = FILL_PLATE_K                               # 먼 판 생선 칸
    cx, cy = t0[0] + md[0] * k, t0[1] + md[1] * k
    rects = _obstacles()
    best = None
    for df, ds in ((0, 0), (8, 0), (-8, 0), (0, 15), (0, -15)):   # 칸 안쪽에서만
        S = [cx + md[0] * df - md[1] * ds, cy + md[1] * df + md[0] * ds, MOLD_TOP_Z + POUR_GAP_CUP]
        for A in range(0, 360, 5):
            if _pour_clear(S, A, rects) < POUR_CLEAR:
                continue
            F, T, H = _pour_poses(S, A)
            try:
                for q in (F, T, H):
                    check_pose(q, "붓기 계획")
            except ValueError:
                continue
            # 그리퍼가 로봇 바깥쪽을 향할 때(자연스러운 자세)만, 팔이 적당히 펴진(≈40cm) 방향 선호
            r = math.hypot(F[0], F[1])
            outward = (F[0] * math.cos(math.radians(A)) + F[1] * math.sin(math.radians(A))) / r
            if outward < -0.6:          # 그리퍼가 로봇 쪽으로 완전히 되돌아보는 자세만 제외
                continue
            fold = min(arm_fold(F), arm_fold(T))
            score = -abs(fold - 450.0) + 30.0 * outward
            if best is None or score > best[0]:
                best = (score, S, float(A))
        if best:
            break
    if best is None:
        raise ValueError("[붓기 계획 실패] 손잡이·소스통을 피해서 부을 방향이 없어요 — 비커/틀 위치를 조정하세요")
    return best[1], best[2]


# ================= 뚜껑 닫기·뒤집기·열기: 1·2번 틀 각자의 손잡이를 '옆에서' 잡기 =================
# 철판 가운데 손잡이는 없음. 틀마다 가까운 판·먼 판에 손잡이가 하나씩 (틀 가운데 줄).
# 1번 틀은 커넥터 쪽(좁은 쪽)에서, 2번 틀은 넓은 쪽에서 옆으로 들어와 잡아요 (옆 틀 손잡이와 안 부딪히게).
SIDE_GRIP_SIGNS = [-1.0, +1.0]
HANDLE_SHORTS = [MOLD_SHORT]     # ★ 손잡이 줄 (철판 가운데, 앞 손잡이·뒤 손잡이 한 쌍)   # ★ 틀별 그리퍼가 들어오는 쪽: -1 = 좁은 쪽, +1 = 넓은 쪽
LID_GRIP_R = 150.0       # ★ 경첩 → 손잡이 잡는 점 거리 (틀에서 떨어진 손잡이 바깥쪽을 잡음)
STACK_T = HANDLE_T       # 손잡이 하나 두께 (겹치면 2배)


def _zyz_mat(a, b, c):
    a, b, c = (math.radians(t) for t in (a, b, c))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    return [[ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb],
            [sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb],
            [-sb * cc, sb * sc, cb]]


def _mat_zyz(R):
    b = math.atan2(math.hypot(R[0][2], R[1][2]), R[2][2])
    if abs(math.sin(b)) < 1e-6:
        a, c = 0.0, math.atan2(R[1][0], R[0][0]) if R[2][2] > 0 else math.atan2(R[1][0], -R[0][0])
    else:
        a = math.atan2(R[1][2], R[0][2])
        c = math.atan2(R[2][1], -R[2][0])
    return [math.degrees(a), math.degrees(b), math.degrees(c)]


def _axis_rot(k, deg):
    t = math.radians(deg); c, s_ = math.cos(t), math.sin(t); x, y, z = k
    return [[c + x * x * (1 - c), x * y * (1 - c) - z * s_, x * z * (1 - c) + y * s_],
            [y * x * (1 - c) + z * s_, c + y * y * (1 - c), y * z * (1 - c) - x * s_],
            [z * x * (1 - c) - y * s_, z * y * (1 - c) + x * s_, c + z * z * (1 - c)]]


def _mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _side_base(mi):
    vy = -SIDE_GRIP_SIGNS[mi] * SHORT_SIGN
    return _zyz_mat(90.0 if vy > 0 else -90.0, 90.0, 90.0), (0.0, vy, 0.0)


def side_lid_pose(theta, z_extra=0.0, mi=0):
    """mi번 틀: 경첩 각 theta(0 = 먼 판 펼쳐짐, 180 = 가까운 판 위에 덮임)일 때 먼 판 손잡이를 옆에서 잡은 자세"""
    R0, v = _side_base(mi)
    hx = MOLD_SIGN * HINGE_FWD
    hy = SHORT_SIGN * HANDLE_SHORTS[mi]
    hz = MOLD_GRIP[2]
    th = math.radians(theta)
    tx = hx + MOLD_SIGN * LID_GRIP_R * math.cos(th)
    tz = hz + LID_GRIP_R * math.sin(th) + z_extra
    R = _mm(_axis_rot((0.0, -MOLD_SIGN, 0.0), theta), R0)
    return [tx - v[0] * TOOL_LENGTH, hy - v[1] * TOOL_LENGTH, tz - v[2] * TOOL_LENGTH, *_mat_zyz(R)]


def side_flip_pose(beta, lift=0.0, mi=0):
    """mi번 틀: 닫힌 뒤 겹친 손잡이를 옆에서 잡고, 손잡이 축(틀 방향)으로 beta도 돌린 자세"""
    R0, v = _side_base(mi)
    tx = MOLD_SIGN * (HINGE_FWD - LID_GRIP_R)
    ty = SHORT_SIGN * HANDLE_SHORTS[mi]
    tz = MOLD_GRIP[2] + STACK_T / 2 + lift
    k = (-1.0 if v[1] > 0 else 1.0, 0.0, 0.0)          # 돌리는 동안 그리퍼 몸통이 '위'로 지나감
    R = _mm(_axis_rot(k, beta), R0)
    u = [R[i][2] for i in range(3)]
    return [tx - u[0] * TOOL_LENGTH, ty - u[1] * TOOL_LENGTH, tz - u[2] * TOOL_LENGTH, *_mat_zyz(R)]


# ---- 반죽·속재료는 '먼 판'에 넣고, 가까운 판을 넘겨 덮음 → 겹친 손잡이가 먼 쪽에 생김 ----
FILL_PLATE_K = HANDLE_STICK + MOLD_W * 1.5      # 가까운 손잡이 잡는 점 → 먼 판 가운데(생선 칸)


def front_flip_pose(lift=0.0):
    """먼 쪽에 겹친 손잡이를 로봇 쪽에서 '정면으로' 잡는 자세 (손가락 위아래로 닫혀 4cm 두께를 잡음)"""
    tcp_fwd = HINGE_FWD + LID_R                  # 먼 손잡이 잡는 점 (판 가장자리에서 4cm)
    return pose_to(tcp_fwd - TOOL_LENGTH, MOLD_SHORT, MOLD_GRIP[2] + STACK_T / 2 + lift)


# ---- 제자리에서 다시 잡기: 닫은 자리에서 그리퍼만 벌렸다 조여 겹친 손잡이를 잡고, 그대로 뒤집고, 그대로 펼침 ----
def side_tcp(theta, z_extra=0.0, mi=0):
    """경첩 각 theta일 때 그 판 손잡이 잡는 점 (0 = 먼 쪽, 180 = 가까운 쪽)"""
    th = math.radians(theta)
    return [MOLD_SIGN * HINGE_FWD + MOLD_SIGN * LID_GRIP_R * math.cos(th),
            SHORT_SIGN * HANDLE_SHORTS[mi],
            MOLD_GRIP[2] + LID_GRIP_R * math.sin(th) + z_extra]


def pose_with(tcp, R):
    """손끝 위치 + 회전행렬 → 로봇 자세"""
    u = [R[i][2] for i in range(3)]
    return [tcp[k] - u[k] * TOOL_LENGTH for k in range(3)] + _mat_zyz(R)


def hinge_rot(theta, R_at_far):
    """먼 쪽(0°)에서의 그리퍼 회전 R_at_far 를 경첩 축으로 theta 만큼 돌린 회전"""
    return _mm(_axis_rot((0.0, -MOLD_SIGN, 0.0), theta), R_at_far)


def handle_rot(beta, R_start, mi=0):
    """손잡이 축(틀 방향)으로 beta 만큼 돌린 회전. 돌리는 동안 그리퍼 몸통이 위로 지나가는 방향"""
    vy = R_start[1][2]
    return _mm(_axis_rot((-1.0 if vy > 0 else 1.0, 0.0, 0.0), beta), R_start)


# ================= 돌림판: 경첩 봉을 잡고 틀을 수평으로 90° 돌린 뒤, 겹친 손잡이를 끝에서 잡아 뒤집기 =================
# 틀 아래 상자 = 돌림판 (중심 = 경첩 가운데). 경첩 봉은 짧은 방향으로 놓여 있고 끝이 철판 밖으로 나와 있음.
ROD_R = 110.0            # ★ 돌림판 중심 → 경첩 봉 끝 잡는 점 (철판 반폭 9cm + 2cm)
ROD_SIDE = -1.0          # ★ 잡는 봉 끝: -1 = 좁은 쪽(커넥터 쪽), +1 = 넓은 쪽
ROD_DIA = 10.0           # ★ 봉 지름 (그림용)


def _turn_center():
    return (MOLD_SIGN * HINGE_FWD, SHORT_SIGN * MOLD_SHORT)


def turn_deg():
    """먼 쪽에 겹친 손잡이가 봉이 있던 쪽(좁은 쪽)으로 오도록 돌리는 각도 (위에서 봤을 때, 로봇 좌표)"""
    a = math.atan2(0.0, MOLD_SIGN)
    b = math.atan2(ROD_SIDE * SHORT_SIGN, 0.0)
    d = math.degrees(b - a)
    return (d + 180.0) % 360.0 - 180.0


def _rot2(v, deg):
    t = math.radians(deg); c, s_ = math.cos(t), math.sin(t)
    return (v[0] * c - v[1] * s_, v[0] * s_ + v[1] * c)


def _end_on(tcp, inward):
    """끝에서(축 방향으로) 잡는 자세: 그리퍼가 inward 방향(수평)을 보고 손가락은 위아래로 닫힘"""
    a = math.degrees(math.atan2(inward[1], inward[0]))
    R = _zyz_mat(a, 90.0, 90.0)
    return pose_with(tcp, R)


def rod_pose(t):
    """돌림판을 t도(0 → turn_deg()) 돌렸을 때 경첩 봉 끝을 '위에서' 잡은 자세
    (그리퍼가 아래를 보고, 손가락이 봉을 가로질러 닫힘 → 팔이 접히지 않음)"""
    cx, cy = _turn_center()
    d = _rot2((0.0, ROD_SIDE * SHORT_SIGN), t)
    tcp = [cx + d[0] * ROD_R, cy + d[1] * ROD_R, HINGE_Z]
    yaw = math.degrees(math.atan2(d[1], d[0]))
    return pose_with(tcp, _zyz_mat(0.0, 180.0, -yaw))


def turned_stack_pose(lift=0.0, z_extra=0.0):
    """돌린 뒤: 겹친 손잡이를 '끝에서' 잡은 자세 (손잡이 축 방향으로 들어감)"""
    cx, cy = _turn_center()
    d = _rot2((MOLD_SIGN, 0.0), turn_deg())          # 먼 쪽이었던 방향이 돌아간 방향
    tcp = [cx + d[0] * LID_GRIP_R, cy + d[1] * LID_GRIP_R, MOLD_GRIP[2] + STACK_T / 2 + lift + z_extra]
    return _end_on(tcp, (-d[0], -d[1]))


def turned_open_tcp(theta, z_extra=0.0):
    """돌린 뒤 펼치기: 위 손잡이 잡는 점이 경첩(이제 틀 방향 축)을 돌아 반대편으로 (0 → 180)"""
    cx, cy = _turn_center()
    d = _rot2((MOLD_SIGN, 0.0), turn_deg())
    th = math.radians(theta)
    return [cx + d[0] * LID_GRIP_R * math.cos(th), cy + d[1] * LID_GRIP_R * math.cos(th),
            MOLD_GRIP[2] + LID_GRIP_R * math.sin(th) + z_extra]


def turned_hinge_rot(theta, R_start):
    """돌린 뒤의 경첩 축(수평, 손잡이와 직각)으로 theta 돌린 회전 — 손잡이가 먼저 위로 들리는 방향"""
    d = _rot2((MOLD_SIGN, 0.0), turn_deg())
    k = (d[1], -d[0], 0.0)                           # (k × d).z = +1 → 손잡이가 위로 먼저 들림
    return _mm(_axis_rot(k, theta), R_start)


def pour_z():
    """주전자로 부을 때 플랜지 목표 z (위에서 잡은 자세, 주둥이 끝이 틀 위 3cm)"""
    return MOLD_TOP_Z + POUR_GAP_CUP + KETTLE_SPOUT_DROP     # 옆잡기(수평)라 플랜지 높이 = 잡는 점 높이


def fill_z():
    """소스통으로 짤 때 플랜지 목표 z (입구 끝이 틀 위 3cm)"""
    return MOLD_TOP_Z + FILL_GAP + NOZZLE_DROP


# 이동 높이 = 가장 높은 물체 위 14cm (자동)
OBJ_TRANSIT_Z = max(SAUCE_BOTTOM_Z + SAUCE_H, TABLE_TOP_Z + CUP_BASE_H + CUP_H, MOLD_TOP_Z) + 140.0

_spec_selfcheck()    # 속도·관절·무게 설정이 사양 안인지 시작할 때 확인
