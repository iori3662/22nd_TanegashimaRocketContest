from picamera2 import Picamera2
import cv2
import numpy as np
import pigpio
import time

# =========================================
# ===== サーボ仕様（あなたの実測事実）====
# =========================================
# 停止値
STOP = 1490

# 入力可能範囲
MIN_PULSE = 500
MAX_PULSE = 2500

# pin18: 小さいほど前進
# pin12: 大きいほど前進

SERVO_LEFT  = 18
SERVO_RIGHT = 12

# 安全のため最大変位を制限（推奨は500程度）
MAX_DELTA = 600

# =========================================
# ===== 制御パラメータ ====================
# =========================================
Kp_turn = 0.8
BASE_FWD = 250
SEARCH_TURN = 200

AREA_TRACK_TH = 900
AREA_FWD_TH = 2200

CENTER_TOL_RATIO = 0.08  # 画面幅の8%以内なら前進

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
    turn > 0 で右旋回

    pin18: 小さいほど前進 → STOP - value
    pin12: 大きいほど前進 → STOP + value
    """

    left_power  = fwd + turn
    right_power = fwd - turn

    left_power  = clamp(left_power,  -MAX_DELTA, MAX_DELTA)
    right_power = clamp(right_power, -MAX_DELTA, MAX_DELTA)

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
    W, H = 640, 480
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "BGR888", "size": (W, H)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.3)

    stop(pi)

    try:
        while True:

            frame = picam2.capture_array("main")
            if frame is None:
                continue

            # 物理的に時計回り90° → ソフトで反時計回り補正
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

            H2, W2 = frame.shape[:2]
            cx_frame = W2 // 2
            center_tol_px = int(W2 * CENTER_TOL_RATIO)

            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

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
                # 見失い
                set_diff_drive(pi, 0, SEARCH_TURN)
                status = "SEARCH"
            else:
                M = cv2.moments(best)
                if M["m00"] == 0:
                    set_diff_drive(pi, 0, SEARCH_TURN)
                    status = "M00=0"
                else:
                    cx = int(M["m10"] / M["m00"])
                    err = cx - cx_frame

                    if abs(err) < center_tol_px:
                        # 中央 → 前進
                        fwd = BASE_FWD if best_area < AREA_FWD_TH else int(BASE_FWD * 0.8)
                        set_diff_drive(pi, fwd, 0)
                        status = "FORWARD"
                    else:
                        turn = int(Kp_turn * err)
                        set_diff_drive(pi, 0, turn)
                        status = "TURN"

            # ---- 表示 ----
            cv2.line(frame, (cx_frame, 0), (cx_frame, H2), (255, 255, 255), 1)

            if best_approx is not None:
                cv2.drawContours(frame, [best_approx], -1, (0,255,0), 2)

            cv2.putText(frame, status, (20,40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

            cv2.imshow("Camera", frame)
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