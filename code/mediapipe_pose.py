import socket
import pickle
import numpy as np
import cv2
import mediapipe as mp
import math
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

def main(use_socket=False, ip="127.0.0.1", port=8000):
  # For webcam input:
  cap = cv2.VideoCapture(0)

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
      mp_drawing.draw_landmarks(
          image,
          results.pose_landmarks,
          mp_pose.POSE_CONNECTIONS,
          landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
      # Flip the image horizontally for a selfie-view display.
      cv2.imshow('Output', cv2.flip(image, 1))

      if results.pose_landmarks:
        data = get_joint_angles(results)
      else:
        data = None
      
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
  left_shoulder=[results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER].y]
  right_shoulder=[results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER].y]
  right_elbow=[results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW].y]
  right_wrist=[results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].x, results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST].y]
  
  lshoul_rshoul = np.array(right_shoulder)-np.array(left_shoulder)
  shoul_elb = np.array(right_elbow)-np.array(right_shoulder)
  elb_wris=np.array(right_wrist)-np.array(right_elbow)


  shoul_elb_rad=math.acos(np.dot(lshoul_rshoul,shoul_elb)/(np.linalg.norm(shoul_elb)*np.linalg.norm(lshoul_rshoul)))
  elb_wris_rad = math.acos(np.dot(shoul_elb,elb_wris)/(np.linalg.norm(shoul_elb)*np.linalg.norm(elb_wris)))
  shoul_elb_deg=math.degrees(shoul_elb_rad)
  elb_wris_deg = math.degrees(elb_wris_rad)
  print(right_wrist)
  return [right_wrist[0],right_wrist[1]]

if __name__ == "__main__":
  use_socket = True
  ip = "127.0.0.1"
  port = 8000
  main(use_socket=use_socket, ip=ip, port=port)
