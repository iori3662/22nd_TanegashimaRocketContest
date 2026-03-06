from picamera2 import Picamera2
import cv2
import numpy as np
import pigpio
import time

# =========================================
# ===== サーボ仕様（あなたの実測事実）====
# =========================================
STOP = 1490
MIN_PULSE = 500
MAX_PULSE = 2500

SERVO_LEFT  = 18   # pin18: 小さいほど前進
SERVO_RIGHT = 12   # pin12: 大きいほど前進

MAX_DELTA = 600

# =========================================
# ===== 制御パラメータ ====================
# =========================================
Kp_turn = 0.8
BASE_FWD = 250
SEARCH_TURN = 200

AREA_TRACK_TH = 900
AREA_FWD_TH = 2200

CENTER_TOL_RATIO = 0.08  # 画面幅の8%以内なら中心一致

# =========================================
# ===== 0mゴール判定（追加）===============
# =========================================
SLOW_RED_RATIO = 0.35     # これ以上で減速モード（接近）
GOAL_RED_RATIO = 0.60     # これ以上で「0mゴール」候補
GOAL_HOLD_SEC  = 0.25     # 連続で超えたら確定（誤停止防止）

# =========================================
# ===== 赤色検出HSV設定 ===================
# =========================================
lower_red1 = np.array([0, 120, 70], dtype=np.uint8)
upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
lower_red2 = np.array([170, 120, 70], dtype=np.uint8)
upper_red2 = np.array([179, 255, 255], dtype=np.uint8)
kernel = np.ones((5, 5), np.uint8)

# =========================================
# ===== ユーティリティ ====================
# =========================================
def clamp(val, lo, hi):
    return max(lo, min(hi, int(val)))

def stop(pi):
    pi.set_servo_pulsewidth(SERVO_LEFT, STOP)
    pi.set_servo_pulsewidth(SERVO_RIGHT, STOP)

def set_diff_drive(pi, fwd, turn):
    """
    fwd  > 0 で前進
    turn > 0 で右旋回（ただし実機に合わせてここで反転済み）
    """
    # 実機挙動に合わせて旋回符号を反転（あなたの確定済み設定）
    turn = -turn

    left_power  = fwd + turn
    right_power = fwd - turn

    left_power  = clamp(left_power,  -MAX_DELTA, MAX_DELTA)
    right_power = clamp(right_power, -MAX_DELTA, MAX_DELTA)

    # pin18: 小さいほど前進 → STOP - power
    # pin12: 大きいほど前進 → STOP + power
    pulse_left  = STOP - left_power
    pulse_right = STOP + right_power

    pulse_left  = clamp(pulse_left,  MIN_PULSE, MAX_PULSE)
    pulse_right = clamp(pulse_right, MIN_PULSE, MAX_PULSE)

    pi.set_servo_pulsewidth(SERVO_LEFT, pulse_left)
    pi.set_servo_pulsewidth(SERVO_RIGHT, pulse_right)

def is_triangle_like(contour, min_area):
    area = cv2.contourArea(contour)
    if area < min_area:
        return False, None, area

    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

    if len(approx) != 3:
        return False, approx, area

    if not cv2.isContourConvex(approx):
        return False, approx, area

    return True, approx, area

# =========================================
# ================= MAIN ==================
# =========================================
def main():

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio未起動。sudo pigpiod を実行してください。")

    # -------- カメラ設定 --------
    W, H = 480, 640
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "BGR888", "size": (W, H)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.3)

    stop(pi)

    # ゴール連続判定用
    goal_start_t = None

    try:
        while True:

            frame = picam2.capture_array("main")
            if frame is None:
                continue

            # 物理的に時計回り90°設置 → ソフトで反時計回り90°補正
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

            H2, W2 = frame.shape[:2]
            cx_frame = W2 // 2
            center_tol_px = int(W2 * CENTER_TOL_RATIO)

            # ★重要：frameはBGRなので BGR2HSV（あなたの元コードはRGB2HSVで不整合）
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # ===== 0mゴール判定（追加）=====
            red_ratio = cv2.countNonZero(mask) / float(H2 * W2)

            if red_ratio >= GOAL_RED_RATIO:
                if goal_start_t is None:
                    goal_start_t = time.time()
                elif (time.time() - goal_start_t) >= GOAL_HOLD_SEC:
                    stop(pi)
                    print(f"GOAL(0m): red_ratio={red_ratio:.3f}")
                    break
            else:
                goal_start_t = None

            # ===== 追跡（遠距離は三角形、近距離は三角形崩壊してもOK）=====
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            best = None
            best_area = 0
            best_approx = None

            for c in contours:
                ok, approx, area = is_triangle_like(c, AREA_TRACK_TH)
                if ok and area > best_area:
                    best = c
                    best_area = area
                    best_approx = approx

            if best is None:
                # 近距離で三角形が崩れても、赤率が高いなら「減速して直進」寄りにする
                if red_ratio >= SLOW_RED_RATIO:
                    set_diff_drive(pi, int(BASE_FWD * 0.35), 0)
                    status = "NEAR(FWD)"
                else:
                    set_diff_drive(pi, 0, SEARCH_TURN)
                    status = "SEARCH"
            else:
                M = cv2.moments(best)
                if M["m00"] == 0:
                    if red_ratio >= SLOW_RED_RATIO:
                        set_diff_drive(pi, int(BASE_FWD * 0.35), 0)
                        status = "NEAR(FWD)"
                    else:
                        set_diff_drive(pi, 0, SEARCH_TURN)
                        status = "M00=0"
                else:
                    cx = int(M["m10"] / M["m00"])
                    err = cx - cx_frame

                    if abs(err) < center_tol_px:
                        # 中心 → 前進（接近したら減速）
                        if red_ratio >= SLOW_RED_RATIO:
                            fwd = int(BASE_FWD * 0.35)
                        else:
                            fwd = BASE_FWD if best_area < AREA_FWD_TH else int(BASE_FWD * 0.8)

                        set_diff_drive(pi, fwd, 0)
                        status = "FORWARD"
                    else:
                        # 旋回（近距離は過敏だと暴れるので、赤率が高いほど旋回量を少し抑える）
                        turn_gain = 1.0
                        if red_ratio >= SLOW_RED_RATIO:
                            turn_gain = 0.6

                        turn = int(turn_gain * Kp_turn * err)
                        set_diff_drive(pi, 0, turn)
                        status = "TURN"

            # ---- 表示 ----
            disp = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            cv2.line(disp, (cx_frame, 0), (cx_frame, H2), (255, 255, 255), 1)

            if best_approx is not None:
                cv2.drawContours(disp, [best_approx], -1, (0, 255, 0), 2)

            cv2.putText(disp, f"{status} red_ratio={red_ratio:.2f}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow("Camera", disp)
            cv2.imshow("Mask", mask)

            if cv2.waitKey(1) & 0xFF in [27, ord('q')]:
                break

    finally:
        stop(pi)
        time.sleep(0.2)
        pi.set_servo_pulsewidth(SERVO_LEFT, 0)
        pi.set_servo_pulsewidth(SERVO_RIGHT, 0)
        pi.stop()
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()