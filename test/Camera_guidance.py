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

# 停止からの最大変位(us)（大きいほど速いが不安定になりやすい）
MAX_DELTA = 350

# =========================
# Control tuning
# =========================
Kp_turn = 0.9          # 旋回ゲイン（0.6〜1.5あたりで調整）
BASE_FWD = 220         # 前進ベース (us)
SEARCH_TURN = 170      # 見失い時の探索旋回 (us)

AREA_TRACK_TH = 800    # この面積以上で「追跡対象あり」
AREA_FWD_TH = 1500     # この面積以上で前進を強める（近づいたら上げる/下げる）

# =========================
# Red detection settings
# =========================
lower_red1 = np.array([0, 120, 70], dtype=np.uint8)
upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
lower_red2 = np.array([170, 120, 70], dtype=np.uint8)
upper_red2 = np.array([179, 255, 255], dtype=np.uint8)
kernel = np.ones((5, 5), np.uint8)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def set_diff_drive(pi, fwd_us, turn_us):
    """
    fwd_us : 前進成分 (+で前進)
    turn_us: 旋回成分 (+で右旋回, -で左旋回) ※逆なら符号を反転して調整
    """
    cmd_l = STOP_L + fwd_us + turn_us
    cmd_r = STOP_R + fwd_us - turn_us

    cmd_l = clamp(cmd_l, STOP_L - MAX_DELTA, STOP_L + MAX_DELTA)
    cmd_r = clamp(cmd_r, STOP_R - MAX_DELTA, STOP_R + MAX_DELTA)

    pi.set_servo_pulsewidth(SERVO_L_PIN, cmd_l)
    pi.set_servo_pulsewidth(SERVO_R_PIN, cmd_r)

def stop(pi):
    pi.set_servo_pulsewidth(SERVO_L_PIN, STOP_L)
    pi.set_servo_pulsewidth(SERVO_R_PIN, STOP_R)

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

    # 初期停止
    stop(pi)
    time.sleep(0.3)

    cx_frame = W // 2

    try:
        while True:
            # Picamera2: "RGB888" で取得 → OpenCV処理用にBGRへ変換
            frame_rgb = picam2.capture_array("main")
            if frame_rgb is None or frame_rgb.size == 0:
                stop(pi)
                continue

            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)

                if area >= AREA_TRACK_TH:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])

                        # 画像中心からの正規化誤差（-1〜+1）
                        err = (cx - cx_frame) / (W / 2.0)

                        # 旋回量(us)
                        turn = int(Kp_turn * err * MAX_DELTA)

                        # 前進量(us)：赤が大きいほど強める（雑に2段階）
                        fwd = BASE_FWD if area < AREA_FWD_TH else int(BASE_FWD * 1.2)

                        set_diff_drive(pi, fwd_us=fwd, turn_us=turn)

                        # ---- 可視化（デバッグ）----
                        x, y, w, h = cv2.boundingRect(c)
                        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.circle(frame_bgr, (cx, int(M["m01"] / M["m00"])), 4, (0, 255, 0), -1)
                        cv2.putText(frame_bgr, f"RED area={int(area)} err={err:.2f}",
                                    (x, max(0, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    else:
                        # モーメントが取れない場合は探索
                        set_diff_drive(pi, fwd_us=0, turn_us=SEARCH_TURN)
                else:
                    # 小さすぎ＝見失い扱い
                    set_diff_drive(pi, fwd_us=0, turn_us=SEARCH_TURN)
            else:
                # 見失い：探索旋回
                set_diff_drive(pi, fwd_us=0, turn_us=SEARCH_TURN)

            # 画面中心線（デバッグ）
            cv2.line(frame_bgr, (cx_frame, 0), (cx_frame, H), (255, 255, 255), 1)

            cv2.imshow("Camera", frame_bgr)
            cv2.imshow("Red Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:  # q or ESC
                break

    finally:
        # 停止 → フリー → 終了
        stop(pi)
        time.sleep(0.2)
        pi.set_servo_pulsewidth(SERVO_L_PIN, 0)
        pi.set_servo_pulsewidth(SERVO_R_PIN, 0)
        pi.stop()

        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()