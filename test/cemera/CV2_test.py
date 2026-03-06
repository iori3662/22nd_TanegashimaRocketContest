from picamera2 import Picamera2
import cv2
import time

W, H = 640, 480

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (W, H)}   # いったんRGBで固定
)
picam2.configure(config)
picam2.start()
time.sleep(0.2)

try:
    while True:
        frame = picam2.capture_array("main")  # これはRGBのはず（設定通りなら）

        # そのまま表示（OpenCVはBGR前提なので色が変に見えるはず）
        cv2.imshow("RAW (as-is)", frame)

        # RGB→BGRにして表示（こっちが現実の色に合うなら入力はRGB確定）
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imshow("RGB->BGR (should look real)", frame_bgr)

        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break
finally:
    picam2.stop()
    cv2.destroyAllWindows()
