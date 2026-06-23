from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerResult
import mediapipe as mp
from mediapipe.tasks import python 
from mediapipe.tasks.python import vision
import cv2
import threading
import time
import os
import numpy as np

from hand_controller import HandController

class FingerCapture:
    modelpath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand_landmarker.task")

    # Define hand connections: (start_index, end_index)
    connections = [
            # Thumb
            (0, 1), (1, 2), (2, 3), (3, 4),
            # Index Finger
            (0, 5), (5, 6), (6, 7), (7, 8),
            # Middle Finger
            (0, 9), (9, 10), (10, 11), (11, 12),
            # Ring Finger
            (0, 13), (13, 14), (14, 15), (15, 16),
            # Pinky
            (0, 17), (17, 18), (18, 19), (19, 20),
            # Knuckle connections (connecting MCP joints)
            (5, 9), (9, 13), (13, 17)
        ]
    

    def __init__(self, num_hands):
        self.isRunning = False

        #Thread stuff
        self.trackingThread = threading.Thread(target=self.processing_callback, daemon=True)
        self.robotThread = threading.Thread(target=self.robot_callback, daemon=True)
        self.frameLock = threading.Lock()
        self.currentFrame = None
        self.result_lock = threading.Lock()
        self.result = None

        #Video stuff
        self.cap = cv2.VideoCapture(0)

        #Robot init
        self.hand_robot = HandController()

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
        with self.result_lock:
            self.result = result

    def start(self):
        self.isRunning = True
        self.trackingThread.start()
        self.robotThread.start()
        self.display_callback()

        self.robotThread.join(timeout=2.0)
        self.trackingThread.join(timeout=2.0)

    
    def processing_callback(self):
        """Takes frame from camera and processes the image to get hand landmarks
        """
        try:
            with self.HandLandmarker.create_from_options(self.LandmarkerOptions) as landmarker:
                consecutive_failures = 0
                while self.isRunning:
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
                    
                    # lock thread and copy over current frame back to main
                    with self.frameLock:
                        self.currentFrame = frame.copy()

                    #convert image to mediapipe image
                    rgb_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_img)

                    #detect landmarks using camera
                    timestamp = int(time.time()*1000)
                    landmarker.detect_async(mp_image, timestamp)

        except Exception as e: 
            print("error in processing, exiting")
            self.isRunning = False

    def robot_callback(self):
        """Handles the control of the robot hand
        """
        thumb = (1,4)
        pointer = (5,8)
        middle = (9,12)
        ring = (13,16)
        pinky = (17,20)

        self.hand_robot.enable(1)

        try:
            while(self.isRunning):
                with self.result_lock:
                    latest_result = self.result
                
                if latest_result is None or not latest_result.hand_landmarks:
                    time.sleep(0.1)
                    continue
                
                #get landmarks and get finger angle
                first_hand_landmarks = latest_result.hand_landmarks[0]
                pointer_angle = self.get_finger_angle(first_hand_landmarks, pointer)

                #convert to motor andle and set
                motor_angle = self.hand_robot.finger2motor(pointer_angle)
                self.hand_robot.setPosition(1, motor_angle)

                time.sleep(0.1)

        finally:
            self.hand_robot.disable(1)
            self.hand_robot.close_port()





    
    def get_finger_angle(self, hand_landmarks, indexes):
        knuckle_idx = indexes[0]
        tip_idx = indexes[1]

        wrist = np.array([hand_landmarks[0].x, hand_landmarks[0].y, hand_landmarks[0].z])
        knuckle = np.array([hand_landmarks[knuckle_idx].x, hand_landmarks[knuckle_idx].y, hand_landmarks[knuckle_idx].z])
        tip = np.array([hand_landmarks[tip_idx].x, hand_landmarks[tip_idx].y, hand_landmarks[tip_idx].z])

        # Calculate vectors
        v_palm = knuckle - wrist
        v_finger = tip - knuckle
        
        # Dot product formula: cos(theta) = (u . v) / (|u| * |v|)
        dot_product = np.dot(v_palm, v_finger)
        norm_palm = np.linalg.norm(v_palm)
        norm_finger = np.linalg.norm(v_finger)
        
        cos_angle = np.clip(dot_product / (norm_palm * norm_finger), -1.0, 1.0)
        angle_deg = np.degrees(np.arccos(cos_angle))
        return angle_deg

    def draw_landmarks_on_image(self, annotated_image, detection_result):
        height, width, _ = annotated_image.shape

        if detection_result.hand_landmarks:
            for hand_landmarks in detection_result.hand_landmarks:
                # Convert normalized coordinates to pixel coordinates
                coords = []
                for landmark in hand_landmarks:
                    cx = int(landmark.x * width)
                    cy = int(landmark.y * height)
                    coords.append((cx, cy))

                # Draw connection lines (Red)
                for start_idx, end_idx in self.connections:
                    if start_idx < len(coords) and end_idx < len(coords):
                        cv2.line(annotated_image, coords[start_idx], coords[end_idx], (0, 0, 255), 2)

                # Draw landmark points (Green)
                for coord in coords:
                    cv2.circle(annotated_image, coord, 5, (0, 255, 0), -1)

        return annotated_image

    def display_callback(self):
        try:
            while self.isRunning:
                frame = None

                # get the current frame
                with self.frameLock:
                    if self.currentFrame is not None:
                        frame = self.currentFrame.copy()

                if frame is None:
                    time.sleep(0.01)
                    continue

                # 1. Safely grab the latest data payload
                with self.result_lock:
                    latest_result = self.result
                    
                if latest_result and latest_result.hand_landmarks:
                    # Convert BGR frame to RGB, draw landmarks, and convert back
                    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    annotated_rgb = self.draw_landmarks_on_image(rgb_frame, latest_result)
                    frame = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)
                    
                cv2.imshow('Video Feed', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Quit key pressed, stopping capture.")
                    break

        finally: 
            self.cap.release()
            cv2.destroyAllWindows()
            self.isRunning = False


if __name__ == "__main__":
    hands = FingerCapture(1)
    hands.start()
