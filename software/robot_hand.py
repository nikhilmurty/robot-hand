from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerResult
import mediapipe as mp
from mediapipe.tasks import python 
from mediapipe.tasks.python import vision
import cv2
import threading
import time
import os

class FingerCapture:
    modelpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_landmarker.task")
    

    def __init__(self, num_hands):
        self.cameraThread = threading.Thread(target=self.tracking_callback, daemon=True)
        self.cap = cv2.VideoCapture(0)

        # set up hand model
        self.HandLandmarker = mp.tasks.vision.HandLandmarker
        BaseOptions = mp.tasks.BaseOptions
        HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
        self.HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult
        VisionRunningMode = mp.tasks.vision.RunningMode

        self.LandmarkerOptions = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path= self.modelpath),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=self.print_result)
        
    def print_result(self, result: HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
        pass
    
    def tracking_callback(self):
        try:
            with self.HandLandmarker.create_from_options(self.LandmarkerOptions) as landmarker:
                consecutive_failures = 0
                while self.cap.isOpened():
                    ret, frame = self.cap.read()
                    
                    #watch to see if camera isn't reading
                    if not ret or frame is None or frame.size == 0:
                        consecutive_failures += 1
                        if consecutive_failures % 10 == 1:
                            print("Waiting for camera feed to initialize...")
                        time.sleep(0.1)
                        if consecutive_failures > 50:
                            print("Error: Could not read frame from camera. Exiting.")
                            break
                        continue
                    consecutive_failures = 0

                    #capture the video image
                    cv2.imshow('Video Feed', frame)

                    #convert image to mediapipe image
                    rgb_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_img)

                    #detect landmarks using camera
                    timestamp = int(time.time()*1000)
                    landmarker.detect_async(mp_image, timestamp)

                    # Press 'q' to exit the loop
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("Quit key pressed, stopping capture")
                        break

        except KeyboardInterrupt: 
            print("Terminal shortcut pressed, exiting")

        finally: 
            self.cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    test = FingerCapture(1)
    test.tracking_callback()
