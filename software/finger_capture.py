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

    def run(self):
        self.isRunning = True
        self.trackingThread.start()
        self.robotThread.start()
        self.hand_robot.start()
        self.display_callback()

        self.robotThread.join(timeout=2.0)
        self.trackingThread.join(timeout=2.0)
        self.hand_robot.stop()

    
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
        thumb   = (0, 2, 4, 15)
        pointer = (0, 5, 8, 15)
        middle  = (0, 9, 12, 8)
        ring    = (0, 13, 16, 4)
        pinky   = (0, 17, 20, 6)

        try:
            while(self.isRunning):
                with self.result_lock:
                    latest_result = self.result
                
                if latest_result is None or not latest_result.hand_landmarks:
                    time.sleep(0.1)
                    continue
                
                #get landmarks and get finger angle
                first_hand_landmarks = latest_result.hand_landmarks[0]
                thumb_angle   = self.get_finger_angle(first_hand_landmarks, thumb)
                pointer_angle = self.get_finger_angle(first_hand_landmarks, pointer)
                middle_angle  = self.get_finger_angle(first_hand_landmarks, middle)
                ring_angle    = self.get_finger_angle(first_hand_landmarks, ring)
                pinky_angle   = self.get_finger_angle(first_hand_landmarks, pinky)

                positions = [thumb_angle, pointer_angle, middle_angle, ring_angle, pinky_angle]

                # print(f"Positions: {positions}")

                #convert to motor andle and set
                self.hand_robot.setHandPosition(positions)

                time.sleep(0.1)

        finally:
            print("Stopping hand controller")

    
    def get_finger_angle(self, hand_landmarks, indexes):
        wrist_idx, mcp_idx, tip_idx, offset = indexes

        wrist = np.array([hand_landmarks[wrist_idx].x, hand_landmarks[wrist_idx].y, hand_landmarks[wrist_idx].z])
        mcp = np.array([hand_landmarks[mcp_idx].x, hand_landmarks[mcp_idx].y, hand_landmarks[mcp_idx].z])
        tip = np.array([hand_landmarks[tip_idx].x, hand_landmarks[tip_idx].y, hand_landmarks[tip_idx].z])

        # Calculate vectors: v_palm (Wrist -> MCP) and v_finger (MCP -> Tip)
        v_palm = mcp - wrist
        v_finger = tip - mcp
        
        # Dot product formula: cos(theta) = (u . v) / (|u| * |v|)
        dot_product = np.dot(v_palm, v_finger)
        norm_palm = np.linalg.norm(v_palm)
        norm_finger = np.linalg.norm(v_finger)
        
        denom = norm_palm * norm_finger
        if denom == 0 or np.isnan(denom):
            return 0.0

        cos_angle = np.clip(dot_product / denom, -1.0, 1.0)
        angle_deg = np.degrees(np.arccos(cos_angle))

        if np.isnan(angle_deg):
            return 0.0

        # Subtract natural baseline rest offset so straight hand maps to 0 deg
        return max(0.0, float(angle_deg - offset))

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
    hands.run()
