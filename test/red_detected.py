from picamera2 import Picamera2
import cv2
import numpy as np
import time

def main():
    W, H = 640, 480
    picam2 = Picamera2()

    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (W, H)}
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)

    lower_red1 = np.array([0, 120, 70], dtype=np.uint8)
    upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_red2 = np.array([170, 120, 70], dtype=np.uint8)
    upper_red2 = np.array([179, 255, 255], dtype=np.uint8)
    kernel = np.ones((5, 5), np.uint8)

    try:
        while True:
            frame_bgr = picam2.capture_array("main") 
            if frame_bgr is None or frame_bgr.size == 0:
                continue

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
                if area > 800:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame_bgr, f"RED area={int(area)}", (x, max(0, y - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Camera", frame_bgr)
            cv2.imshow("Red Mask", mask)

            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
