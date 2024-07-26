import socket
import pickle
import numpy as np
import cv2
import mediapipe as mp
from mediapipe.python.solutions.pose import PoseLandmark
import math
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose
 # list of wanted landmarks 
wanted_pose_landmarks = [
  PoseLandmark.RIGHT_WRIST, 
    ]

is_hand_in_frame = False

def main(use_socket=False, ip="127.0.0.1", port=8000):
  # For webcam input:
  cap = cv2.VideoCapture(0)
  is_hand_in_frame = False

  if use_socket:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip, port))

  with mp_pose.Pose(
      min_detection_confidence=0.5,
      min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
      success, image = cap.read()
      if not success:
        print("Ignoring empty camera frame.")
        # If loading a video, use 'break' instead of 'continue'.
        continue

      # To improve performance, optionally mark the image as not writeable to
      # pass by reference.
      image.flags.writeable = False
      image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
      results = pose.process(image)

      # Draw the pose annotation on the image.
      image.flags.writeable = True
      image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
      
      # Draw grid on image
      height, width, channels = image.shape 
      cv2.line(image,(0,int(height/2)),(int(width),int(height/2)),(255,0,0),5)
      cv2.line(image,(int(width/2),0),(int(width/2),int(height)),(255,0,0),5)

      #Draw landmark on the wrist
      if results.pose_landmarks:
                for landmark in wanted_pose_landmarks:
                    landmark_pos = results.pose_landmarks.landmark[landmark.value]
                    x, y = int(landmark_pos.x * width), int(landmark_pos.y * height)
                    cv2.circle(image, (x, y), 30, (0, 255, 0), -1)

      if results.pose_landmarks:
        data = get_joint_angles(results)
      else:
        data = None
      
      image = cv2.flip(image, 1)
      # Border for signalization if the hand is in the frame
      if not data[2]: # not in the frame
        image = cv2.copyMakeBorder(image, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=[0, 0, 255]) 
        
        sizeText = cv2.getTextSize("PUT RIGHT HAND IN TO THE FRAME", cv2.FONT_HERSHEY_PLAIN, 3, 2)[0]

        cv2.rectangle(image, (int(width/2) - (sizeText[0]//2) - 6, int(height/2) + sizeText[1] + 6),
                                  (int(width/2) + (sizeText[0]//2) + 6, int(height/2) - sizeText[1]//2 - 6), (0, 0, 255),
                                  thickness=cv2.FILLED)
        cv2.putText(image, "PUT RIGHT HAND IN TO THE FRAME", (int(width/2) - (sizeText[0]//2), (int(height/2)+sizeText[1])),
                    cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 255), 2)
      else:
        image = cv2.copyMakeBorder(image, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=[0, 255, 0]) 
      
      # Flip the image horizontally for a selfie-view display.
      cv2.imshow('Output', image)

      if use_socket:
        serialized_data = pickle.dumps(data)
        client_socket.send(serialized_data)  

      if cv2.waitKey(5) & 0xFF == 27:
        break

  cap.release()

  if use_socket:
    client_socket.close()

def get_joint_angles(results):
  # Add your code here
  # For example, to get right shoulder location, use [results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].y]
  # See link below for location of body parts
  # https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
  '''
  left_shoulder=[results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].y]
  right_shoulder=[results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].y]
  right_elbow=[results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].y]
  '''
  right_wrist=[results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].y]
  global is_hand_in_frame
  if results.pose_landmarks:
        is_hand_in_frame = (
            0 <= right_wrist[0] <= 1 and
            0 <= right_wrist[1] <= 1 
        )
  else:
        is_hand_in_frame = False
  #check if the right wrist is in the frame

  return [right_wrist[0],right_wrist[1],is_hand_in_frame]

if __name__ == "__main__":
  use_socket = False
  ip = "127.0.0.1"
  port = 8000
  main(use_socket=use_socket, ip=ip, port=port)
