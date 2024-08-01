import socket
import pickle
import time
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
  start_time = time.time()
  goal_is_reached = False
  game_started_time = 0
  showTotalTime = False
  total_time = 0
  if use_socket:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ip, port))

  with mp_pose.Pose(
      min_detection_confidence=0.5,
      min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
      success, image = cap.read()
      height, width, channels = image.shape 
      if not success:
        print("Ignoring empty camera frame.")
        # If loading a video, use 'break' instead of 'continue'.
        continue
      
      if (time.time() - start_time) < 3: # the first 5s - name of the game
        data = [0,0,0]
        image[:] = (0, 0, 0)
        sizeText = cv2.getTextSize("PYTHON MAZE CRAZE", cv2.FONT_HERSHEY_PLAIN, 2, 2)[0]
        cv2.putText(image, "PYTHON MAZE CRAZE", (int(width/2) - (sizeText[0]//2), (int(height/2)+sizeText[1])),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
      elif (time.time() - start_time) >= 3 and (time.time() - start_time) < 4:
        data = [0,0,0]
        image[:] = (0, 0, 0)
        sizeText = cv2.getTextSize("START IN ", cv2.FONT_HERSHEY_PLAIN, 2, 2)[0]
        cv2.putText(image, "START IN ", (int(width/2) - (sizeText[0]//2), (int(height/2)+sizeText[1])),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
      elif (time.time() - start_time) >= 4 and (time.time() - start_time) < 5:
        data = [0,0,0]
        image[:] = (0, 0, 0)
        sizeText = cv2.getTextSize("3", cv2.FONT_HERSHEY_PLAIN, 4, 2)[0]
        cv2.putText(image, "3", (int(width/2) - (sizeText[0]//2), (int(height/2)+sizeText[1])),cv2.FONT_HERSHEY_PLAIN, 4, (255, 255, 255), 2)
      elif (time.time() - start_time) >= 5 and (time.time() - start_time) < 6:
        data = [0,0,0]
        image[:] = (0, 0, 0)
        sizeText = cv2.getTextSize("2", cv2.FONT_HERSHEY_PLAIN, 4, 2)[0]
        cv2.putText(image, "2", (int(width/2) - (sizeText[0]//2), (int(height/2)+sizeText[1])),cv2.FONT_HERSHEY_PLAIN, 4, (255, 255, 255), 2)
      elif (time.time() - start_time) >= 6 and (time.time() - start_time) < 7:
        data = [0,0,0]
        image[:] = (0, 0, 0)
        sizeText = cv2.getTextSize("1", cv2.FONT_HERSHEY_PLAIN, 4, 2)[0]
        cv2.putText(image, "1", (int(width/2) - (sizeText[0]//2), (int(height/2)+sizeText[1])),cv2.FONT_HERSHEY_PLAIN, 4, (255, 255, 255), 2)
      else:
        if (game_started_time == 0):
          game_started_time = time.time()

        if not goal_is_reached:
          image.flags.writeable = False
          image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
          results = pose.process(image)

          # Draw the pose annotation on the image.
          image.flags.writeable = True
          image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
          
          # Draw grid on image
          cv2.line(image,(0,int(height/2)),(int(width),int(height/2)),(128,128,128),5)
          cv2.line(image,(int(width/2),0),(int(width/2),int(height)),(128,128,128),5)

          #Draw landmark on the wrist
          if results.pose_landmarks:
            for landmark in wanted_pose_landmarks:
                landmark_pos = results.pose_landmarks.landmark[landmark.value]
                x, y = int(landmark_pos.x * width), int(landmark_pos.y * height)
                cv2.circle(image, (x, y), 30, (0, 255, 0), -1)

          if results.pose_landmarks:
            data = get_joint_angles(results)
          else:
            data = [0,0,0]
          
          # Flip the image horizontally for a selfie-view display.
          image = cv2.flip(image, 1)
          
          # Gaming time
          time_text = f'TIME: {(time.time() - game_started_time):.2f}'
          sizeText = cv2.getTextSize(time_text, cv2.FONT_HERSHEY_PLAIN, 1, 1)[0]
          cv2.rectangle(image, (int(width * 0.85) - (sizeText[0]//2) - 6, int(height * 0.1) + sizeText[1] + 6),
                                    (int(width * 0.85) + (sizeText[0]//2) + 6, int(height * 0.1) - sizeText[1]//2 - 6), (0, 255, 0),
                                    thickness=cv2.FILLED)
          cv2.putText(image,time_text, (int(width * 0.85) - (sizeText[0]//2), (int(height * 0.1) +sizeText[1])),
                      cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1)
          
          # tutorial arrows
          if (time.time() - start_time) >= 7 and (time.time() - start_time) < 17:
            #arrow pointing down
            start_point = (int(width * 0.1), int(height * 0.55))
            end_point = (int(width * 0.1), int(height * 0.55) + int(height * 0.2))
            color = (0, 255, 0)  
            thickness = 3
            cv2.arrowedLine(image, start_point, end_point, color, thickness, tipLength=0.2)
            # text next to the arrow
            sizeText = cv2.getTextSize("BACKWARD", cv2.FONT_HERSHEY_PLAIN, 1, 1)[0]
            cv2.rectangle(image, (int(width * 0.1) - (sizeText[0]//2) - 6, int(height * 0.55) + (int(height * 0.25)) + sizeText[1] + 6),
                                      (int(width * 0.1) + (sizeText[0]//2) + 6, int(height * 0.55) + int(height * 0.25) - sizeText[1]//2 - 6), (0, 255, 0),
                                      thickness=cv2.FILLED)
            cv2.putText(image, "BACKWARD", (int(width * 0.1) - (sizeText[0]//2), (int(height * 0.55) + int(height * 0.25) +sizeText[1])),
                        cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1)

            #arrow pointing up 
            start_point = (int(width * 0.1), int(height * 0.45))
            end_point = (int(width * 0.1), int(height * 0.45) - int(height * 0.2))
            color = (0, 255, 0)  
            thickness = 3
            cv2.arrowedLine(image, start_point, end_point, color, thickness, tipLength=0.2)
            # text next to the arrow
            sizeText = cv2.getTextSize("FORWARD", cv2.FONT_HERSHEY_PLAIN, 1, 1)[0]
            cv2.rectangle(image, (int(width * 0.1) - (sizeText[0]//2) - 6, int(height * 0.45) - (int(height * 0.25)) + sizeText[1] + 6),
                                      (int(width * 0.1) + (sizeText[0]//2) + 6, int(height * 0.45) - int(height * 0.25) - sizeText[1]//2 - 6), (0, 255, 0),
                                      thickness=cv2.FILLED)
            cv2.putText(image, "FORWARD", (int(width * 0.1) - (sizeText[0]//2), (int(height * 0.45) - int(height * 0.25) +sizeText[1])),
                        cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1)
            #arrow pointing left 
            start_point = (int(width * 0.45), int(height * 0.9))
            end_point = (int(width * 0.45) - int(height * 0.2), int(height * 0.9))
            color = (0, 255, 0) 
            thickness = 3
            cv2.arrowedLine(image, start_point, end_point, color, thickness, tipLength=0.2)
            # text next to the arrow
            sizeText = cv2.getTextSize("LEFT", cv2.FONT_HERSHEY_PLAIN, 1, 1)[0]
            cv2.rectangle(image, (int(width * 0.45) - int(height * 0.2) - (sizeText[0]//2) - 6, int(height * 0.9) - (int(height * 0.08)) + sizeText[1] + 6),
                                      (int(width * 0.45) - int(height * 0.2) + (sizeText[0]//2) + 6, int(height * 0.9) - int(height * 0.08) - sizeText[1]//2 - 6), (0, 255, 0),
                                      thickness=cv2.FILLED)
            cv2.putText(image, "LEFT", (int(width * 0.45) - int(height * 0.2) - (sizeText[0]//2), (int(height * 0.9) - int(height * 0.08) +sizeText[1])),
                        cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1)
            
            #arrow pointing right
            start_point = (int(width * 0.55), int(height * 0.9))
            end_point = (int(width * 0.55) + int(height * 0.2), int(height * 0.9))
            color = (0, 255, 0)  
            thickness = 3
            cv2.arrowedLine(image, start_point, end_point, color, thickness, tipLength=0.2)
            # text next to the arrow
            sizeText = cv2.getTextSize("RIGHT", cv2.FONT_HERSHEY_PLAIN, 1, 1)[0]
            cv2.rectangle(image, (int(width * 0.55) + int(height * 0.2) - (sizeText[0]//2) - 6, int(height * 0.9) - (int(height * 0.08)) + sizeText[1] + 6),
                                      (int(width * 0.55) + int(height * 0.2) + (sizeText[0]//2) + 6, int(height * 0.9) - int(height * 0.08) - sizeText[1]//2 - 6), (0, 255, 0),
                                      thickness=cv2.FILLED)
            cv2.putText(image, "RIGHT", (int(width * 0.55) + int(height * 0.2) - (sizeText[0]//2), (int(height * 0.9) - int(height * 0.08) +sizeText[1])),
                        cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 1)
          # Border for signalization if the hand is in the frame
          if not data[2]: # not in the frame
            image = cv2.copyMakeBorder(image, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=[0, 0, 255]) 
            
            sizeText = cv2.getTextSize("PUT RIGHT HAND IN TO THE FRAME", cv2.FONT_HERSHEY_PLAIN, 1.5, 2)[0]

            cv2.rectangle(image, (int(width/2) - (sizeText[0]//2) - 6, int(height/2) + sizeText[1] + 6),
                                      (int(width/2) + (sizeText[0]//2) + 6, int(height/2) - sizeText[1]//2 - 6), (0, 0, 255),
                                      thickness=cv2.FILLED)
            cv2.putText(image, "PUT RIGHT HAND IN TO THE FRAME", (int(width/2) - (sizeText[0]//2), (int(height/2)+sizeText[1])),
                        cv2.FONT_HERSHEY_PLAIN, 1.5, (255, 255, 255), 2)
          else:
            image = cv2.copyMakeBorder(image, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=[0, 255, 0]) 
        else:
          if not showTotalTime:
            total_time = time.time() - game_started_time
            showTotalTime = True
          data = [0,0,0]
          image[:] = (0, 0, 0)
          sizeTextG = cv2.getTextSize("SUCCESS", cv2.FONT_HERSHEY_PLAIN, 2, 2)[0]
          cv2.putText(image, "SUCCESS", (int(width/2) - (sizeTextG[0]//2), (int(height/2)+sizeTextG[1])),cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)

          time_text = f'TOTAL TIME: {(total_time):.2f}'
          sizeText = cv2.getTextSize(time_text, cv2.FONT_HERSHEY_PLAIN, 2, 1)[0]
          cv2.putText(image,time_text, (int(width/2) - (sizeText[0]//2), (int(height/2)+sizeText[1] + int(sizeTextG[1]*2))),
                      cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255), 2)
      cv2.imshow('Output', image)

      if use_socket:
        serialized_data = pickle.dumps(data)
        client_socket.send(serialized_data)  
        receive = client_socket.recv(4096)
        goal_is_reached = pickle.loads(receive)
        
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

  return [right_wrist[0],right_wrist[1],is_hand_in_frame]

if __name__ == "__main__":
  use_socket = False
  ip = "127.0.0.1"
  port = 8000
  main(use_socket=use_socket, ip=ip, port=port)
