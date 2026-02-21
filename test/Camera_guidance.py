from picamera2 import Picamera2
import cv2
import numpy as np
import pigpio
import time

# =========================
# Servo pins / calibration
# =========================
SERVO_L_PIN = 18  # GPIO18 (Left)
SERVO_R_PIN = 12  # GPIO12 (Right)

STOP_L = 1480
STOP_R = 1490

MAX_DELTA = 990

# =========================
# Control tuning
# =========================
Kp_turn = 0.9
BASE_FWD = 220
SEARCH_TURN = 170

AREA_TRACK_TH = 900     # 三角形の面積しきい値（環境で調整）
AREA_FWD_TH = 2200      # これ以上なら前進強め

# =========================
# Red detection settings (HSV)
# =========================
lower_red1 = np.array([0, 120, 70], dtype=np.uint8)
upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
lower_red2 = np.array([170, 120, 70], dtype=np.uint8)
upper_red2 = np.array([179, 255, 255], dtype=np.uint8)
kernel = np.ones((5, 5), np.uint8)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def set_diff_drive(pi, fwd_us, turn_us):
    # turn_us: +で右旋回（逆なら符号反転で調整）
    cmd_l = STOP_L + fwd_us + turn_us
    cmd_r = STOP_R + fwd_us - turn_us

    cmd_l = clamp(cmd_l, STOP_L - MAX_DELTA, STOP_L + MAX_DELTA)
    cmd_r = clamp(cmd_r, STOP_R - MAX_DELTA, STOP_R + MAX_DELTA)

    pi.set_servo_pulsewidth(SERVO_L_PIN, cmd_l)
    pi.set_servo_pulsewidth(SERVO_R_PIN, cmd_r)

def stop(pi):
    pi.set_servo_pulsewidth(SERVO_L_PIN, STOP_L)
    pi.set_servo_pulsewidth(SERVO_R_PIN, STOP_R)

def is_triangle_like(contour, min_area=AREA_TRACK_TH):
    """
    赤コーンの「三角形っぽさ」を輪郭で判定。
    - 面積が小さすぎるものは捨てる
    - approxPolyDPで頂点が3なら三角形扱い
    - ついでに「細すぎ/潰れすぎ」を除外（バウンディング比）
    """
    area = cv2.contourArea(contour)
    if area < min_area:
        return False, None, area

    peri = cv2.arcLength(contour, True)
    eps = 0.04 * peri  # 0.03〜0.06で調整（小さいほど厳密）
    approx = cv2.approxPolyDP(contour, eps, True)

    if len(approx) != 3:
        return False, approx, area

    x, y, w, h = cv2.boundingRect(approx)
    if h == 0:
        return False, approx, area
    aspect = w / h

    # 三角形（コーン）はだいたい縦長〜正方に近いはず、極端な横長を除外
    if aspect > 1.6:
        return False, approx, area

    # 凸性チェック（必須ではないが誤検出低減）
    if not cv2.isContourConvex(approx):
        return False, approx, area

    return True, approx, area

def main():
    # ---- pigpio ----
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpioに接続できません。sudo pigpiod を実行してください。")

    # ---- Picamera2 ----
    W, H = 640, 480
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (W, H)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)

    stop(pi)
    time.sleep(0.3)

    cx_frame = W // 2

    try:
        while True:
            frame_rgb = picam2.capture_array("main")
            if frame_rgb is None or frame_rgb.size == 0:
                stop(pi)
                continue

            # ★重要：判定はRGB→HSVでやる（BGR2HSVにしない）
            hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 三角形だけ抽出して最大をターゲットに
            best = None
            best_area = 0
            best_approx = None

            for c in contours:
                ok, approx, area = is_triangle_like(c)
                if ok and area > best_area:
                    best = c
                    best_area = area
                    best_approx = approx

            if best is None:
                # 見失い：探索旋回
                set_diff_drive(pi, fwd_us=0, turn_us=SEARCH_TURN)
                target_info = "NO TRIANGLE"
            else:
                M = cv2.moments(best)
                if M["m00"] <= 0:
                    set_diff_drive(pi, fwd_us=0, turn_us=SEARCH_TURN)
                    target_info = "M00=0"
                else:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    err = (cx - cx_frame) / (W / 2.0)
                    turn = int(Kp_turn * err * MAX_DELTA)

                    fwd = BASE_FWD if best_area < AREA_FWD_TH else int(BASE_FWD * 1.2)
                    set_diff_drive(pi, fwd_us=fwd, turn_us=turn)

                    target_info = f"TRI area={int(best_area)} err={err:.2f}"

            # ---- 表示用：RGB→BGRにしてimshow（これで赤青逆転が直る）----
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            # デバッグ描画
            cv2.line(frame_bgr, (cx_frame, 0), (cx_frame, H), (255, 255, 255), 1)
            cv2.putText(frame_bgr, target_info, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if best_approx is not None:
                cv2.drawContours(frame_bgr, [best_approx], -1, (0, 255, 0), 2)

            cv2.imshow("Camera", frame_bgr)
            cv2.imshow("Red Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break

    finally:
        stop(pi)
        time.sleep(0.2)
        pi.set_servo_pulsewidth(SERVO_L_PIN, 0)
        pi.set_servo_pulsewidth(SERVO_R_PIN, 0)
        pi.stop()
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()