from __future__ import annotations

import time
from dataclasses import dataclass

import cv2
import numpy as np
import pigpio
from picamera2 import Picamera2


# ============================================================
# 0) サーボ・機体の「実測仕様」（あなたの確定済み事実）
# ============================================================
# - 停止は 1490us 付近
# - 左サーボ(pin18): 値が小さいほど前進
# - 右サーボ(pin12): 値が大きいほど前進
STOP_US = 1490
MIN_PULSE_US = 500
MAX_PULSE_US = 2500

SERVO_LEFT_PIN = 18   # BCM18
SERVO_RIGHT_PIN = 12  # BCM12

# “差分駆動”として許容する最大の変化量（過大だと暴れる）
MAX_DELTA_US = 600


# ============================================================
# 1) 制御パラメータ（現場で調整するやつ）
# ============================================================
# 旋回の比例ゲイン（画面中心からのズレ[px] -> 旋回コマンド）
KP_TURN = 0.8

# 前進量（差分駆動の “fwd” に相当：大きいほど速い）
BASE_FWD = 250

# 目標が見つからないときの探索旋回量
SEARCH_TURN = 200

# 三角形とみなす最小面積（遠距離は三角形で追跡しやすい）
AREA_TRACK_TH = 900

# “十分近い” とみなす面積（これ以上なら少し減速）
AREA_FWD_TH = 2200

# 目標中心の許容ズレ（画面幅に対する割合）
CENTER_TOL_RATIO = 0.08  # 画面幅の8%以内なら中心扱い


# ============================================================
# 2) 「0mゴール」判定（赤が画面を占める割合で停止する）
# ============================================================
# 三角形が崩れる距離では「形状」より「赤の面積割合」が安定する想定
SLOW_RED_RATIO = 0.35   # これ以上なら“近い”→減速モード
GOAL_RED_RATIO = 0.60   # これ以上が一定時間続けば“ゴール”
GOAL_HOLD_SEC = 0.25    # 誤停止防止：連続で超えたら確定


# ============================================================
# 3) 赤色検出（HSV）
# ============================================================
# OpenCVのHSV: Hは [0,179]
# 赤は 0付近と 179付近に分かれるので2レンジ合成が基本
LOWER_RED1 = np.array([0, 120, 70], dtype=np.uint8)
UPPER_RED1 = np.array([10, 255, 255], dtype=np.uint8)
LOWER_RED2 = np.array([170, 120, 70], dtype=np.uint8)
UPPER_RED2 = np.array([179, 255, 255], dtype=np.uint8)

# ノイズ除去用（開閉処理）
KERNEL = np.ones((5, 5), np.uint8)


# ============================================================
# 4) 小物ユーティリティ
# ============================================================
def clamp_int(x: float, lo: int, hi: int) -> int:
    """整数として範囲に収める（サーボ指令で使う）"""
    return int(max(lo, min(hi, int(x))))


def make_red_mask(bgr: np.ndarray) -> np.ndarray:
    """
    BGR画像 -> 赤領域の2値マスク(0/255)
    IMPORTANT:
      Picamera2 を BGR888 で取っているなら変換は BGR2HSV が正しい
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 小さなゴミを落として、穴を埋める
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
    return mask


def contour_is_triangle_like(contour, min_area: int):
    """
    “三角形っぽい”かの判定。
    - 面積が小さいものは無視
    - 近似多角形が3頂点、かつ凸
    """
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


# ============================================================
# 5) 差動サーボ駆動（あなたの実測方向に合わせた変換）
# ============================================================
class DiffDrive:
    """
    fwd/turn から左右サーボのパルス幅に変換する。
    注意:
      - 左(pin18)は “小さいほど前進”
      - 右(pin12)は “大きいほど前進”
    """

    def __init__(self, pi: pigpio.pi):
        self.pi = pi
        self.stop()

    def stop(self):
        self.pi.set_servo_pulsewidth(SERVO_LEFT_PIN, STOP_US)
        self.pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, STOP_US)

    def free(self):
        # pigpioのサーボ出力解除（0にすると停止とは違う点に注意）
        self.pi.set_servo_pulsewidth(SERVO_LEFT_PIN, 0)
        self.pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, 0)

    def set(self, fwd: int, turn: int):
        """
        fwd  > 0 で前進
        turn > 0 で右旋回（※実機に合わせて符号はここで調整済みにする）

        ここで turn を反転しているのは「あなたの実機で右旋回の符号が逆だった」ため。
        """
        turn = -turn  # 実機挙動に合わせて符号反転（確定済み）

        left_power = fwd + turn
        right_power = fwd - turn

        left_power = clamp_int(left_power, -MAX_DELTA_US, MAX_DELTA_US)
        right_power = clamp_int(right_power, -MAX_DELTA_US, MAX_DELTA_US)

        # 左(pin18): 小さいほど前進 → STOP - power
        # 右(pin12): 大きいほど前進 → STOP + power
        pulse_left = STOP_US - left_power
        pulse_right = STOP_US + right_power

        pulse_left = clamp_int(pulse_left, MIN_PULSE_US, MAX_PULSE_US)
        pulse_right = clamp_int(pulse_right, MIN_PULSE_US, MAX_PULSE_US)

        self.pi.set_servo_pulsewidth(SERVO_LEFT_PIN, pulse_left)
        self.pi.set_servo_pulsewidth(SERVO_RIGHT_PIN, pulse_right)


# ============================================================
# 6) カメラ誘導（後で統合しやすい step() 形式）
# ============================================================
@dataclass
class CameraGuidanceStatus:
    mode: str
    red_ratio: float
    best_area: float | None
    arrived: bool


class CameraGuidance:
    """
    画像から赤三角形（遠距離）を追跡し、近距離は赤率でゴール判定する誘導器。
    """

    def __init__(self, drive: DiffDrive):
        self.drive = drive
        self._goal_start_t: float | None = None

    def step(self, frame_bgr: np.ndarray) -> CameraGuidanceStatus:
        """
        1フレーム分の処理を実行し、サーボ指令を出す。
        戻り値はデバッグ/統合用の状態。
        """
        # 物理的に時計回り90°設置 → ソフトで反時計回り90°補正
        frame = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)

        h, w = frame.shape[:2]
        cx_frame = w // 2
        center_tol_px = int(w * CENTER_TOL_RATIO)

        # 赤抽出
        mask = make_red_mask(frame)

        # 赤率（画面に占める赤マスクの割合）
        red_ratio = cv2.countNonZero(mask) / float(h * w)

        # ---- 0mゴール判定（赤率が一定以上を一定時間継続）----
        if red_ratio >= GOAL_RED_RATIO:
            if self._goal_start_t is None:
                self._goal_start_t = time.time()
            elif (time.time() - self._goal_start_t) >= GOAL_HOLD_SEC:
                self.drive.stop()
                return CameraGuidanceStatus(
                    mode="GOAL",
                    red_ratio=red_ratio,
                    best_area=None,
                    arrived=True,
                )
        else:
            self._goal_start_t = None

        # ---- 輪郭抽出して「三角形っぽい」最大を選ぶ（遠距離用）----
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        best_area = 0.0
        best_approx = None

        for c in contours:
            ok, approx, area = contour_is_triangle_like(c, AREA_TRACK_TH)
            if ok and area > best_area:
                best_contour = c
                best_area = float(area)
                best_approx = approx

        # ---- 誘導ロジック ----
        if best_contour is None:
            # 三角形が取れない：
            # - 近距離なら「減速して直進」寄り（赤率が高いので形状が崩れる想定）
            # - 遠距離なら探索旋回
            if red_ratio >= SLOW_RED_RATIO:
                self.drive.set(int(BASE_FWD * 0.35), 0)
                mode = "NEAR_FWD"
            else:
                self.drive.set(0, SEARCH_TURN)
                mode = "SEARCH"

            return CameraGuidanceStatus(mode=mode, red_ratio=red_ratio, best_area=None, arrived=False)

        # 三角形が取れた：重心で中心ズレを計算
        M = cv2.moments(best_contour)
        if M["m00"] == 0:
            # 極端に細い/壊れた輪郭など
            if red_ratio >= SLOW_RED_RATIO:
                self.drive.set(int(BASE_FWD * 0.35), 0)
                mode = "NEAR_FWD"
            else:
                self.drive.set(0, SEARCH_TURN)
                mode = "BAD_MOMENT"
            return CameraGuidanceStatus(mode=mode, red_ratio=red_ratio, best_area=best_area, arrived=False)

        cx = int(M["m10"] / M["m00"])
        err_px = cx - cx_frame

        if abs(err_px) < center_tol_px:
            # 中心に入った → 前進（近距離なら減速）
            if red_ratio >= SLOW_RED_RATIO:
                fwd = int(BASE_FWD * 0.35)
            else:
                # 面積が大きい（近い）ときは少し抑える
                fwd = BASE_FWD if best_area < AREA_FWD_TH else int(BASE_FWD * 0.8)

            self.drive.set(fwd, 0)
            mode = "FORWARD"
        else:
            # 中心からズレている → 旋回
            # 近距離（赤率高）ほど旋回を抑えて暴れを防ぐ
            turn_gain = 0.6 if red_ratio >= SLOW_RED_RATIO else 1.0
            turn = int(turn_gain * KP_TURN * err_px)

            self.drive.set(0, turn)
            mode = "TURN"

        return CameraGuidanceStatus(mode=mode, red_ratio=red_ratio, best_area=best_area, arrived=False)


# ============================================================
# 7) Standalone main（単体試験）
# ============================================================
def main():
    # ---- pigpio 接続 ----
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio未起動です。sudo pigpiod を実行してください。")

    drive = DiffDrive(pi)
    guidance = CameraGuidance(drive)

    # ---- カメラ設定 ----
    # ここで指定した "BGR888" により、frameはBGRになる
    W, H = 480, 640
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "BGR888", "size": (W, H)})
    picam2.configure(config)
    picam2.start()
    time.sleep(0.3)

    # ---- 表示用（任意）----
    show_debug = True

    try:
        while True:
            frame = picam2.capture_array("main")
            if frame is None:
                continue

            st = guidance.step(frame)

            # ゴール確定
            if st.arrived:
                print(f"GOAL(0m): red_ratio={st.red_ratio:.3f}")
                break

            # デバッグ表示（遅いなら show_debug=False にする）
            if show_debug:
                frame_rot = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                mask = make_red_mask(frame_rot)

                h, w = frame_rot.shape[:2]
                cx_frame = w // 2
                disp = cv2.cvtColor(frame_rot, cv2.COLOR_BGR2RGB)
                cv2.line(disp, (cx_frame, 0), (cx_frame, h), (255, 255, 255), 1)

                cv2.putText(
                    disp,
                    f"{st.mode} red_ratio={st.red_ratio:.2f}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2,
                )

                cv2.imshow("Camera", disp)
                cv2.imshow("Mask", mask)

                if cv2.waitKey(1) & 0xFF in [27, ord("q")]:
                    break

    finally:
        # ---- 安全停止 ----
        drive.stop()
        time.sleep(0.2)
        drive.free()

        pi.stop()
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()