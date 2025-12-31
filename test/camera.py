from picamera2 import Picamera2
import cv2
import time

def main():
    picam2 = Picamera2()

    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)}
    )
    picam2.configure(config)

    picam2.start()
    time.sleep(0.2)  

    try:
        while True:
            frame = picam2.capture_array() 
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            cv2.imshow("Picamera2 (OpenCV)", frame_bgr)


            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
