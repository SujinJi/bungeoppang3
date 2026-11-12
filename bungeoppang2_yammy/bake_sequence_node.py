"""붕어빵 전체 공정 — 최종 버전 (두산 E0509 사양 준수)

  (서빙 모드) 손님이 말을 걸면 ㄱ자 인사 / 주문 인식 → 끄덕 / 못 알아들음 → 도리도리
  0 홈
  1 1차 반죽: 로봇 뒤 보조 테이블의 비커(9cm 상자 위, 손잡이 바깥쪽)를 잡아
              → 쓰는 틀마다 '먼 판' 노란 칸 위에서 25° 기울여 2초 → 제자리
  2 속재료:   뒤 테이블의 소스통을 잡아 → 틀 위에서 한 번 뒤집고 → 칸 안쪽 6cm 짜기 → 제자리
  3 2차 반죽: 1과 같고 1초
  4 닫기:     가까운 손잡이를 옆에서 잡아 경첩 축으로 180° 넘겨 먼 판 위에 덮기
  5 10초 굽기
  6 돌림판:   경첩 봉 끝을 위에서 잡아 틀을 수평으로 90° 돌리기
    뒤집기:   겹친 손잡이를 끝에서 잡아 +30cm → 손목 180° → 내려놓기 → 10초
  7 펼치기:   그 자리에서 위 손잡이만 다시 잡아 180° 펼치기 → 돌림판 원위치(−90°)

  모든 이동 전: 안전 영역·작업반경 850mm·팔 접힘(경로 중간까지)·관절 범위(J3·J5 ±150°) 검사
  물체를 들 때: 그리퍼+물체 무게 ≤ 5kg 검사 / 시작할 때: 속도 설정이 사양 안인지 검사

값 수정: grip_params.py(힘·폭) / positions.py(위치·높이·무게·시간)

실행:
  단독 1회:      ros2 run bungeoppang2 bake_sequence [팥|슈크림 ...]
  중간부터:      ros2 run bungeoppang2 bake_sequence 팥 --start=4      (단계 번호는 위 0~7)
  실물:          ros2 run bungeoppang2 bake_sequence --real            (단계마다 Enter)
  주문 대기:     ros2 run bungeoppang2 bake_sequence --serve
"""
import math
import queue as _queue
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, String
import DR_init

from bungeoppang2_yammy import grip_params as G
from bungeoppang2_yammy import positions as P

ROBOT_ID = "dsr01"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = "e0509"


def main(args=None):
    real = "--real" in sys.argv
    step = real and "--no-step" not in sys.argv
    scale = P.SPEED_SCALE_REAL if real else 1.0

    rclpy.init(args=args)
    node = rclpy.create_node("bungeoppang_bake", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    # 드라이버가 완전히 준비될 때까지 기다림 (너무 일찍 연결하면 응답 없이 굳음)
    try:
        import time as _t
        from dsr_msgs2.srv import SetRobotMode as _SRM
        _cli = node.create_client(_SRM, "/" + ROBOT_ID + "/system/set_robot_mode")
        _n = 0
        while not _cli.wait_for_service(timeout_sec=1.0):
            _n += 1
            if _n % 5 == 0:
                node.get_logger().info(f"로봇 드라이버 기다리는 중... ({_n}초)")
        node.get_logger().info("로봇 드라이버 확인 — 안정될 때까지 5초 기다려요")
        _t.sleep(5.0)
        node.destroy_client(_cli)
    except Exception as e:
        node.get_logger().warning(f"드라이버 확인 생략({e})")
    from DSR_ROBOT2 import (movej, movel, posj, posx, get_current_posj, get_current_posx,
                            set_robot_mode, wait, ROBOT_MODE_AUTONOMOUS, DR_MV_MOD_REL)

    grip_node = rclpy.create_node("bungeoppang_gripper")
    ex = SingleThreadedExecutor()
    ex.add_node(grip_node)
    threading.Thread(target=ex.spin, daemon=True).start()
    from bungeoppang2_yammy.gripper_ctrl import GripperCtrl
    gripper = GripperCtrl(grip_node, real=real)
    log = node.get_logger()

    # 총괄(orchestrator) 연동
    done_pub = grip_node.create_publisher(Bool, "/bungeoppang/bake_done", 10)
    held_pub = grip_node.create_publisher(String, "/bungeoppang/held", 10)   # RViz: 잡은 물체 따라오기
    orders = _queue.Queue()
    gestures = _queue.Queue()
    grip_node.create_subscription(String, "/bungeoppang/start_bake",
                                  lambda m: orders.put(m.data), 10)
    grip_node.create_subscription(String, "/bungeoppang/gesture",
                                  lambda m: gestures.put(m.data), 10)

    # ---------------- 공통 도우미 ----------------
    def confirm(msg):
        log.info(f"▶ {msg}")
        if step:
            input("   Enter = 실행 / Ctrl+C = 중단 ")

    def offset_tool(pose, dist):
        """그리퍼가 향하는 방향(틀 쪽)으로 dist만큼 옮긴 자세. 음수 = 뒤로"""
        zx, zy, zz = P.tool_dir(pose)
        q = list(pose)
        q[0] += dist * zx; q[1] += dist * zy; q[2] += dist * zz
        return q

    _ik = {"warned": False}

    def joint_check(pose, label):
        """로봇 자체 역기구학(ikin)으로 목표 관절값을 구해 J1~J6 사양 범위 확인 (특히 J3·J5 ±155°)"""
        try:
            from DSR_ROBOT2 import ikin, get_current_solution_space
            res = ikin(posx(*pose), int(get_current_solution_space()))
            q = res[0] if (isinstance(res, (tuple, list)) and len(res) == 2 and hasattr(res[0], "__len__")) else res
            q = [float(v) for v in list(q)[:6]]
        except Exception as e:
            if not _ik["warned"]:
                log.warning(f"ikin 사용 불가({e}) — 관절 범위는 이동 후 실제 값으로 확인")
                _ik["warned"] = True
            return
        bad = P.joint_problems(q)
        if bad:
            raise ValueError(f"[안전 정지] {label}: 관절 사양 초과 예상 — " + ", ".join(bad))

    def joints_now_check(label):
        bad = P.joint_problems(list(get_current_posj()))
        if bad:
            raise ValueError(f"[안전 정지] {label} 후 관절이 사양 여유를 넘음 — " + ", ".join(bad))

    def move_l(pose, label, vel=None):
        P.check_pose(pose, label)                 # 안전 검사 (positions.py)
        c0 = cur()
        for k in (0.5, 1.0):                      # 가는 길 중간·끝 관절 범위 (로봇 역기구학)
            joint_check([c0[i] + (pose[i] - c0[i]) * k for i in range(3)] + list(pose[3:6]), label)
        P.check_path(cur(), pose, label)          # 가는 길 중간도 검사
        movel(posx(*pose), vel=(vel or P.VEL_L) * scale, acc=P.ACC_L * scale)
        joints_now_check(label)
        got = cur()                               # 실제로 도착했는지 확인
        err = math.dist(got[:3], pose[:3])
        if err > P.REACH_TOL:
            raise ValueError(f"[도달 실패] {label}: 목표와 {err:.0f}mm 차이 "
                             f"(목표 {[round(v) for v in pose[:3]]}, 실제 {[round(v) for v in got[:3]]}) "
                             "→ 로봇이 이 자세로 갈 수 없어요. positions.py에서 위치를 조정하세요")

    def go_rel_j6(deg, vel=None):
        j = list(get_current_posj())
        if abs(j[5] + deg) > P.J6_LIMIT:
            deg = -deg
            if abs(j[5] + deg) > P.J6_LIMIT:
                raise ValueError(f"[안전 정지] J6={j[5]:.0f}°: 회전 불가")
        movej(posj(0, 0, 0, 0, 0, deg), vel=(vel or P.VEL_FLIP) * scale,
              acc=(vel or P.VEL_FLIP) * scale, mod=DR_MV_MOD_REL)
        return deg

    def cur():
        pos, _ = get_current_posx()
        return list(pos)

    def lift_clear():
        """관절 이동(movej) 전에 손끝을 작업대 위 안전 높이로 먼저 올리기.
        movej는 관절을 직선 보간해서 손끝이 어디로 지나갈지 보장이 없어요.
        낮은 위치에서 바로 movej를 하면 팔이 작업대나 물체를 쓸고 지나갈 수 있어서,
        큰 자세 변경 전에는 항상 수직으로 먼저 들어 올려요."""
        try:
            c = cur()
            safe_z = P.TRANSIT_FREE_Z + 50.0
            if c[2] < safe_z:
                c[2] = safe_z
                move_l(c, "자세 바꾸기 전 들어올리기", vel=P.VEL_SLOW_L)
        except Exception:
            pass          # 시작 직후 등 현재 자세를 못 읽으면 그냥 진행

    def home(raise_first=True):
        if raise_first:
            lift_clear()
        movej(posj(*P.HOME_J), vel=P.VEL_J * scale, acc=P.ACC_J * scale)

    def move_jx(pose, label):
        """관절 보간으로 이동 (1축이 자연스럽게 돌아감).
        지금 팔 자세(해)로 못 가는 곳이면 손목 뒤집기 → 어깨 방향 → 팔꿈치 순서로 다른 자세를 시도"""
        P.check_pose(pose, label)
        try:
            from DSR_ROBOT2 import movejx, get_current_solution_space
            c = int(get_current_solution_space())
        except Exception as e:
            log.warning(f"movejx 사용 불가({e}) → 직선 이동")
            movel(posx(*pose), vel=P.VEL_L * scale, acc=P.ACC_L * scale)
            c = None
        if c is not None:
            order = []
            for x in (0, 1, 4, 5):                  # 같은 자세 → 손목만 → 어깨 (팔꿈치는 안 뒤집음: 팔꿈치가 아래로 가면 베이스 아래로 내려감)
                if (c ^ x) not in order:
                    order.append(c ^ x)
            for sol in order:
                movejx(posx(*pose), vel=P.VEL_J * scale, acc=P.ACC_J * scale, sol=sol)
                if math.dist(cur()[:3], pose[:3]) <= P.REACH_TOL:
                    if sol != c:
                        log.info(f"   (팔 자세를 바꿔서 도착: 해 {c} → {sol})")
                    break
        got = cur()
        err = math.dist(got[:3], pose[:3])
        if err > P.REACH_TOL:
            raise ValueError(f"[도달 실패] {label}: 목표와 {err:.0f}mm 차이 "
                             f"(목표 {[round(v) for v in pose[:3]]}, 실제 {[round(v) for v in got[:3]]})")

    def pick(pose, open_w, grip_w, current, label, obj=None, side=0.0):
        confirm(f"{label} 잡기")
        gripper.move(open_w, wait=0.3)
        if obj and side > 0:
            # 옆 손잡이(비커): 접근점 위 → 수직으로 내려와 → 수평으로 다가가 잡기
            home()
            appr = offset_tool(pose, -side)
            high = list(appr); high[2] = P.OBJ_TRANSIT_Z
            move_jx(high, f"{label} 접근점 위 (1축 회전)")
            move_jx(appr, f"{label} 옆으로 내려가기")
            move_l(pose, f"{label} 파지점", vel=P.VEL_SLOW_L)
        elif obj:
            # 세워진 통: 바로 위 → 손가락 벌린 채 수직으로 내려와 감싸 잡기
            home()
            high = list(pose); high[2] = max(pose[2] + P.APPROACH_BACK, P.OBJ_TRANSIT_Z)
            move_jx(high, f"{label} 바로 위 (1축 회전)")
            move_jx(pose, f"{label} 파지점으로 내려가기")
        else:
            back = P.HANDLE_APPROACH_BACK
            while back > 0:                        # 접근점이 안전 영역 밖이면 접근 거리를 줄임
                try:
                    P.check_pose(offset_tool(pose, -back), label)
                    break
                except ValueError:
                    back -= 10.0
            move_l(offset_tool(pose, -max(back, 0.0)), f"{label} 접근")
            move_l(pose, f"{label} 파지점", vel=P.VEL_SLOW_L)
        gripper.move(grip_w, current=current)     # ★ 조임값: grip_params.py
        if obj:
            P.check_payload(obj, label)
            held_pub.publish(String(data=obj + "|" + ",".join(f"{v:.2f}" for v in cur())))

    def place(pose, open_w, label):
        confirm(f"{label} 제자리에 내려놓기")
        above = list(pose); above[2] = max(pose[2] + P.RETREAT_UP, P.OBJ_TRANSIT_Z)
        move_l(above, f"{label} 위")
        move_jx(pose, f"{label} 내려놓기 (바닥에 맞닿게)")
        gripper.move(open_w, wait=0.3)            # 느슨하게 풀기
        held_pub.publish(String(data=""))         # 물체 내려놓음
        above = list(pose); above[2] = max(pose[2] + P.RETREAT_UP, P.OBJ_TRANSIT_Z) 
        move_l(above, f"{label}에서 위로 빠지기", vel=P.VEL_SLOW_L)

    # ---------------- 제스처 (인사 · 끄덕 · 도리도리) ----------------
    def bow():
        confirm("손님 인사: ㄱ자 자세")
        home()                       # 항상 홈에서 인사 시작 (낮은 자세에서 바로 숙이면 위험)
        movej(posj(*P.BOW_J), vel=P.VEL_J * scale, acc=P.ACC_J * scale)
        wait(1.0)
        home(raise_first=False)

    def wiggle(axis, label):
        """그리퍼를 axis(0=x 좌우, 2=z 위아래)로 흔들기"""
        confirm(label)
        base = cur()
        for _ in range(P.GESTURE_REPEAT):
            for d in (+P.GESTURE_AMP, -P.GESTURE_AMP):
                q = list(base); q[axis] += d
                move_l(q, label, vel=P.VEL_L)
        move_l(base, "원위치")

    def gesture(kind):
        if kind == "greet":
            bow()
        elif kind == "yes":
            wiggle(2, "주문 인식: 그리퍼 위아래 끄덕")
        elif kind == "no":
            wiggle(1, "못 알아들음: 그리퍼 좌우 도리도리")   # 좌우 = 짧은 방향(Y)

    # ---------------- 반죽 붓기 (주전자) ----------------
    def pour_round(hold_sec, tag, molds):
        """비커 손잡이를 옆에서 잡아 들고 → 각 틀의 노란 칸 위에 주둥이를 대고 25° 기울여 붓기.
        틀마다 손잡이·소스통을 피하는 방향을 자동으로 골라요 (positions.plan_pour)."""
        K = P.KETTLE_PICK
        plans = [(i, *P.plan_pour(grip)) for i, grip in molds]      # 출발 전에 미리 계획 (실패하면 바로 알림)
        pick(K, G.KETTLE_OPEN_WIDTH, G.KETTLE_GRIP_WIDTH, G.KETTLE_GRIP_CURRENT,
             "비커 손잡이", obj="kettle", side=P.CUP_APPROACH)
        up = cur(); up[2] = P.OBJ_TRANSIT_Z
        move_l(up, "비커 수직으로 들기", vel=P.VEL_SLOW_L)

        for i, S, A in plans:
            F, T, H = P._pour_poses(S, A)
            move_jx(H, f"{tag}: {i+1}번 틀 위로 (그리퍼 방향 {A:.0f}°)")
            move_l(F, f"{tag}: 주둥이를 {i+1}번 노란 칸 위로", vel=P.VEL_SLOW_L)
            confirm(f"{tag}: {P.POUR_TILT_DEG:.0f}도 기울이기 → {hold_sec:.0f}초")
            move_l(T, "주둥이 끝을 고정한 채 기울이기", vel=P.VEL_SLOW_L)
            wait(hold_sec)                                             # ★ 양 조절: 유지 시간
            move_l(F, "바닥면 수평 복귀", vel=P.VEL_SLOW_L)
            move_l(H, "들어올리기")

        confirm("비커 제자리로")
        hk = list(K); hk[2] = P.OBJ_TRANSIT_Z
        move_jx(hk, "비커 자리 위로 (1축 회전)")
        move_jx(K, "비커 내려놓기 (바닥에 맞닿게)")
        gripper.move(G.KETTLE_OPEN_WIDTH, wait=0.3)
        held_pub.publish(String(data=""))
        move_l(offset_tool(K, -P.CUP_APPROACH), "손잡이에서 옆으로 빠지기", vel=P.VEL_SLOW_L)
        out = offset_tool(K, -P.CUP_APPROACH); out[2] = P.OBJ_TRANSIT_Z
        move_l(out, "위로 빠지기", vel=P.VEL_SLOW_L)

    # ---------------- 속재료 (소스통) ----------------
    def dispense_round(kind, molds):
        """소스통 옆면을 잡아 들고, 첫 칸 위에서 한 번만 뒤집은 뒤 그 자세 그대로 옆 칸으로 옮겨 짜기"""
        kind = kind if kind in P.SAUCE_PICKS else "팥"
        bottle = P.SAUCE_PICKS[kind]
        pick(bottle, G.SAUCE_OPEN_WIDTH, G.SAUCE_GRIP_WIDTH,
             G.SAUCE_GRIP_CURRENT, f"{kind} 소스통 옆면 (들 수 있을 만큼만)", obj=f"sauce:{kind}")
        high_z = max(cur()[2] + P.SAUCE_LIFT, P.OBJ_TRANSIT_Z)
        up = cur(); up[2] = high_z
        move_l(up, "입구가 하늘을 향한 채 들기", vel=P.VEL_SLOW_L)

        half = P.FISH_L / 2 - P.DISPENSE_INSET         # 생선 칸(8cm) 가장자리 1cm 안쪽
        z_fl = P.fill_z() + P.SAUCE_GRIP_TO_MOUTH      # 노즐 끝이 틀 위 2cm
        ys = P.SHORT_SIGN * P.DISPENSE_DIR * half
        centers = []
        for i, grip in molds:
            md, tip = P.tool_dir(grip), P.tool_tip(grip)
            centers.append((i, [tip[k] + md[k] * P.FILL_PLATE_K for k in range(2)]))
        # 그리퍼 방향은 첫 칸 기준으로 한 번만 정함 (판 중앙 방향과 90°, 팔이 덜 접히게)
        c0 = centers[0][1]
        ang = math.degrees(math.atan2(c0[1], c0[0]))
        a = min((ang + 90.0, ang - 90.0), key=lambda t: abs((t - bottle[3] + 180.0) % 360.0 - 180.0))
        u = P.tool_dir([0, 0, 0, a, 90.0, 0.0])
        o = None
        for n, (i, c) in enumerate(centers):
            fx, fy = c[0] - u[0] * P.TOOL_LENGTH, c[1] - u[1] * P.TOOL_LENGTH
            if n == 0:
                move_jx([fx, fy + ys, high_z, a, 90.0, 0.0], f"{i+1}번 칸 위로 (1축 회전)")
                confirm("입구를 틀 쪽(아래)으로 뒤집기 (한 번만)")
                go_rel_j6(180.0)
                o = cur()[3:]
            else:
                move_l([fx, fy + ys, high_z, *o], f"뒤집은 자세 그대로 옆 {i+1}번 칸 위로")
            move_l([fx, fy + ys, z_fl, *o], f"{i+1}번 칸 위 2cm", vel=P.VEL_SLOW_L)
            confirm(f"{kind} 짜기({i+1}번): 아래 → 위 ({2*half:.0f}mm)")
            gripper.move(G.SAUCE_SQUEEZE_WIDTH, wait=0.2)  # ★ 짜는 양: grip_params.py
            move_l([fx, fy - ys, z_fl, *o], "짜면서 이동", vel=P.POUR_SPEED)
            gripper.move(G.SAUCE_GRIP_WIDTH, wait=0.2)
            move_l([fx, fy - ys, high_z, *o], "들어올리기")

        confirm("입구를 다시 위로")
        go_rel_j6(-180.0)
        back = list(bottle); back[2] = high_z
        move_jx(back, "제자리 위로 (1축 회전)")
        place(bottle, G.SAUCE_OPEN_WIDTH, f"{kind} 소스통")

    # ---------------- 뚜껑 닫기·뒤집기·열기 (1·2번 틀 각자의 손잡이를 옆에서 잡기) ----------------
    def lid_arc(mi, t0, t1, label, z0=0.0, z1=0.0):
        n = max(1, int(round(abs(t1 - t0) / P.LID_STEP_DEG)))
        for k in range(1, n + 1):
            t = t0 + (t1 - t0) * k / n
            move_l(P.side_lid_pose(t, z0 + (z1 - z0) * k / n, mi), f"{label} {t:.0f}°", vel=P.VEL_SLOW_L)

    def _side_back(pose, label):
        """옆에서 다가가는 거리: 안전 영역 밖이면 10mm씩 줄임"""
        back = P.HANDLE_APPROACH_BACK
        while back > 10:
            try:
                P.check_pose(offset_tool(pose, -back), label)
                break
            except ValueError:
                back -= 10.0
        return back

    def side_approach(pose, label):
        appr = offset_tool(pose, -_side_back(pose, label))
        high = list(appr); high[2] = P.OBJ_TRANSIT_Z
        move_jx(high, f"{label} 옆 위")
        move_jx(appr, f"{label} 옆으로 내려가기")
        move_l(pose, f"{label} 잡는 점", vel=P.VEL_SLOW_L)

    def go_up_safe(label):
        """이동 높이까지 위로. 팔이 너무 펴지는 높이면 닿는 만큼만 올라감"""
        here = cur()
        z = P.OBJ_TRANSIT_Z
        while z > here[2] + 20.0:
            c = list(here); c[2] = z
            try:
                P.check_pose(c, label); P.check_path(here, c, label)
                move_l(c, label, vel=P.VEL_SLOW_L)
                return
            except ValueError:
                z -= 20.0

    def retreat_safe(label):
        """놓은 자리에서 빠지기: 뒤로 → 위로 → 로봇 쪽으로 중 팔이 편한 쪽으로"""
        here = cur()
        r = math.hypot(here[0], here[1]) or 1.0
        cands = [offset_tool(here, -50.0), here[:2] + [here[2] + 60.0] + here[3:],
                 [here[0] - here[0] / r * 60.0, here[1] - here[1] / r * 60.0] + here[2:]]
        for c in cands:
            try:
                P.check_pose(c, label); P.check_path(here, c, label)
                move_l(c, label, vel=P.VEL_SLOW_L)
                break
            except ValueError:
                continue
        go_up_safe("위로 빠지기")

    def side_leave(label):
        here = cur()
        out = offset_tool(here, -_side_back(here, label))
        move_l(out, f"{label}에서 옆으로 빠지기", vel=P.VEL_SLOW_L)
        go_up_safe("위로 빠지기")

    def _R_now():
        return P._zyz_mat(*cur()[3:6])

    def close_lid(mi):
        """가까운 손잡이를 옆에서 잡고 180° 넘겨 먼 판(반죽) 위에 덮기 → 놓고 빠지기"""
        confirm("닫기: 가까운 손잡이를 옆에서 잡기")
        gripper.move(G.HANDLE_OPEN_WIDTH, wait=0.3)
        side_approach(P.side_lid_pose(180.0, 0.0, mi), "가까운 손잡이")
        gripper.move(G.HANDLE_GRIP_WIDTH, current=G.HANDLE_GRIP_CURRENT)
        P.check_payload("lid", "틀 들기"); held_pub.publish(String(data="near|" + ",".join(f"{v:.2f}" for v in cur())))
        confirm("경첩을 축으로 180° 넘겨 먼 판 위에 덮기")
        lid_arc(mi, 180.0, 0.0, "닫기", 0.0, P.STACK_T)
        gripper.move(G.HANDLE_OPEN_WIDTH, wait=0.3)
        held_pub.publish(String(data="|" + ",".join(f"{v:.2f}" for v in cur())))
        side_leave("닫은 손잡이")

    def _end_on_approach(pose, label, open_w):
        gripper.move(open_w, wait=0.3)
        appr = offset_tool(pose, -_side_back(pose, label))
        high = list(appr); high[2] = P.OBJ_TRANSIT_Z
        move_jx(high, f"{label} 위")
        move_jx(appr, f"{label} 앞으로 내려가기")
        move_l(pose, f"{label} 잡는 점", vel=P.VEL_SLOW_L)

    def turn_table(t0, t1, label):
        """경첩 봉 끝을 잡고 돌림판을 t0 → t1 도 수평으로 돌리기"""
        confirm(f"{label}: 경첩 봉 끝 잡기")
        _end_on_approach(P.rod_pose(t0), "경첩 봉 끝", G.HANDLE_OPEN_WIDTH)
        gripper.move(G.ROD_GRIP_WIDTH if hasattr(G, "ROD_GRIP_WIDTH") else 6.0, current=G.HANDLE_GRIP_CURRENT)
        held_pub.publish(String(data="turn|" + ",".join(f"{v:.2f}" for v in cur())))    # RViz: 돌림판 위 틀이 같이 돎
        confirm(f"{label}: 돌림판 {t1 - t0:+.0f}° 돌리기")
        n = max(1, int(round(abs(t1 - t0) / 15.0)))
        for k in range(1, n + 1):
            move_l(P.rod_pose(t0 + (t1 - t0) * k / n), f"돌리기 {(t1 - t0) * k / n:+.0f}°", vel=P.VEL_SLOW_L)
        gripper.move(G.HANDLE_OPEN_WIDTH, wait=0.3)
        held_pub.publish(String(data="|" + ",".join(f"{v:.2f}" for v in cur())))
        side_leave("경첩 봉")

    def flip_stack(mi):
        """돌림판 90° → 겹친 손잡이를 끝에서 잡고 +30cm → 손목 180°(손잡이 축) → 내려놓기 → 10초"""
        turn_table(0.0, P.turn_deg(), "틀 돌리기")
        confirm("뒤집기: 겹친 손잡이를 끝에서 잡기")
        _end_on_approach(P.turned_stack_pose(), "겹친 손잡이", G.STACK_OPEN_WIDTH)
        gripper.move(G.STACK_GRIP_WIDTH, current=G.STACK_GRIP_CURRENT)
        P.check_payload("mold", "틀 들기"); held_pub.publish(String(data="mold|" + ",".join(f"{v:.2f}" for v in cur())))
        move_l(P.turned_stack_pose(P.LIFT_HEIGHT), f"+{P.LIFT_HEIGHT/10:.0f}cm 들어올리기", vel=P.VEL_SLOW_L)
        confirm("공중에서 손목 180° 돌려 뒤집기 (손잡이 축)")
        go_rel_j6(180.0)
        down = cur(); down[2] -= P.LIFT_HEIGHT
        move_l(down, f"그대로 -{P.LIFT_HEIGHT/10:.0f}cm 내리기", vel=P.VEL_SLOW_L)
        confirm(f"잡은 채 {P.BAKE_SEC_SIDE2:.0f}초 대기")
        wait(P.BAKE_SEC_SIDE2)
        held_pub.publish(String(data="|" + ",".join(f"{v:.2f}" for v in cur())))

    def open_lid(mi):
        """그 자리에서 위 손잡이만 다시 잡고 180° 펼치기 → 돌림판 원위치"""
        here = P.tool_tip(cur())
        stack = P.tool_tip(P.turned_stack_pose())
        if math.dist(here, stack) < 30.0:               # 뒤집기 직후: 그 자리에서 다시 잡기
            confirm("그 자리에서 그리퍼 벌렸다가 위 손잡이만 다시 잡기")
            gripper.move(G.STACK_OPEN_WIDTH, wait=0.3)
            top = cur(); top[2] += P.STACK_T / 2
            move_l(top, "위 손잡이 가운데로", vel=P.VEL_SLOW_L)
        else:                                            # 중간 단계부터 시작: 위 손잡이로 다가가기
            confirm("위 손잡이를 끝에서 잡기")
            _end_on_approach(P.turned_stack_pose(z_extra=P.STACK_T / 2), "위 손잡이", G.HANDLE_OPEN_WIDTH)
        gripper.move(G.HANDLE_GRIP_WIDTH, current=G.HANDLE_GRIP_CURRENT)
        Rf = _R_now()
        P.check_payload("lid", "틀 들기"); held_pub.publish(String(data="far|" + ",".join(f"{v:.2f}" for v in cur())))
        end = min(180.0, P.OPEN_RELEASE_DEG)          # 180°는 도달 불가 → 여기까지 펼치고 놓으면 중력이 마저 눕힘
        confirm(f"경첩을 축으로 {end:.0f}° 펼치기 (그 뒤는 놓아서 중력으로)")
        n = max(1, int(round(end / P.LID_STEP_DEG)))
        for k in range(1, n + 1):
            t = end * k / n
            move_l(P.pose_with(P.turned_open_tcp(t, P.STACK_T * (1 - t / 180.0)), P.turned_hinge_rot(t, Rf)),
                   f"열기 {t:.0f}°", vel=P.VEL_SLOW_L)
        gripper.move(G.HANDLE_OPEN_WIDTH, wait=0.3)
        held_pub.publish(String(data="|" + ",".join(f"{v:.2f}" for v in cur())))
        up1 = cur(); up1[2] += 60.0                       # 펼친 뒤엔 뒤로 빼지 않고 위로 (팔이 덜 펴짐)
        move_l(up1, "펼친 손잡이에서 위로 빠지기", vel=P.VEL_SLOW_L)
        turn_table(P.turn_deg(), 0.0, "틀 원위치")

    # ---------------- 전체 시퀀스 ----------------
    def run_all(fillings, start=0):
        """start = 시작할 단계 번호 (중간부터 다시 할 때)
        0 홈 · 1 1차 반죽 · 2 속재료 · 3 2차 반죽 · 4 뚜껑 닫기 · 5 굽기 대기 · 6 돌림판+뒤집기 · 7 펼치기+원위치"""
        STEPS = ["홈", "1차 반죽", "속재료", "2차 반죽", "뚜껑 닫기", "굽기 대기", "돌림판+뒤집기", "펼치기+원위치"]
        n_molds = min(len(fillings) if fillings else 1, len(P.MOLD_GRIPS))
        molds = [(i, P.MOLD_GRIPS[i]) for i in range(max(n_molds, 1))]
        kinds = []
        for i, _ in molds:
            f = fillings[i] if i < len(fillings) and fillings[i] in P.SAUCE_PICKS else "팥"
            kinds.append(f)
        log.info("틀 배정: " + ", ".join(f"{i+1}번={kinds[i]}" for i, _ in molds))
        held_pub.publish(String(data="reset"))          # RViz: 틀을 처음(펼침) 모양으로
        if start:
            log.info(f"▶ {start}단계({STEPS[min(start, len(STEPS) - 1)]})부터 시작")

        def do(n):
            return start <= n

        if do(0):
            confirm("홈 자세")
            home()
            gripper.move(G.HANDLE_OPEN_WIDTH, wait=0.2)
        if do(1):
            pour_round(P.POUR_HOLD1, "1차 반죽", molds)
        if do(2):
            for kind in dict.fromkeys(kinds):          # 주문 순서, 중복 없이
                use = [(i, g) for i, g in molds if kinds[i] == kind]
                dispense_round(kind, use)
        if do(3):
            pour_round(P.POUR_HOLD2, "2차 반죽", molds)
        if do(4):
            for h in range(len(P.HANDLE_SHORTS)):
                close_lid(h)
        if do(5):
            confirm(f"{P.BAKE_SEC_SIDE1:.0f}초 대기")
            wait(P.BAKE_SEC_SIDE1)
        if do(6):
            for h in range(len(P.HANDLE_SHORTS)):
                flip_stack(h)
        if do(7):
            for h in range(len(P.HANDLE_SHORTS)):
                open_lid(h)
        home()
        log.info("✅ 오늘 공정 완료 (틀이 평평하게 열림)")

    try:
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        try:   # 기본값은 특이점(손목이 일직선) 근처에서 멈춤 → 속도만 줄이고 계속 가도록
            from DSR_ROBOT2 import set_singularity_handling
            set_singularity_handling(2)           # 0=회피, 1=멈춤, 2=속도 줄이며 계속
            log.info("특이점 처리: 속도 줄이며 계속")
        except Exception as e:
            log.warning(f"특이점 처리 설정 실패({e}) — 기본값(멈춤) 사용")
        if "--serve" in sys.argv:
            log.info("로봇 연결 확인: 홈 자세로 이동...")
            home(raise_first=False)                 # 여기서 움직이면 드라이버 연결 OK
            log.info("✅ 로봇 연결 확인 완료")
            log.info("대기 중... (start_bake / gesture)")
            while rclpy.ok():
                try:
                    gesture(gestures.get_nowait())
                    continue
                except _queue.Empty:
                    pass
                try:
                    order = orders.get(timeout=0.3)
                except _queue.Empty:
                    continue
                log.info(f"🧾 주문 수신: {order}")
                wait(0.5)                                  # 인사·끄덕 제스처가 먼저 끝나도록
                while True:
                    try:
                        gesture(gestures.get_nowait())
                    except _queue.Empty:
                        break
                fillings = [f.strip() for f in order.split(",") if f.strip()]
                try:
                    run_all(fillings or ["팥"])
                    done_pub.publish(Bool(data=True))
                except ValueError as e:                   # 안전 정지/도달 실패 → 알리고 계속 대기
                    log.error(str(e))
                    held_pub.publish(String(data=""))
                    done_pub.publish(Bool(data=False))
                    try:
                        home()
                    except Exception:
                        pass
                log.info("대기 중...")
        else:
            fillings = [a for a in sys.argv[1:] if not a.startswith("--") and a in P.SAUCE_PICKS]
            st = 0
            for a in sys.argv:
                if a.startswith("--start="):
                    st = int(a.split("=", 1)[1])
            run_all(fillings or ["팥"], start=st)

    except ValueError as e:
        log.error(str(e))
    except KeyboardInterrupt:
        log.warning("사용자가 중단했어요")
    finally:
        try:
            ex.shutdown()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass                                  # Ctrl+C로 이미 종료된 경우


if __name__ == "__main__":
    main()
