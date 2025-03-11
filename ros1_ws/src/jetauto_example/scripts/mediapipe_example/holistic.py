#!/usr/bin/env python3
# encoding: utf-8
import faulthandler
faulthandler.enable()
import os
import cv2
import sys
import mediapipe as mp
import jetauto_sdk.fps as fps

os.system('sudo ~/jetauto_ws/src/jetauto_example/scripts/color_detect/reset.sh')
mp_drawing = mp.solutions.drawing_utils
try:
    mp_drawing_styles = mp.solutions.drawing_styles
except:
    print('***please run workon mediapipe and run this scripts***')
    sys.exit()

mp_holistic = mp.solutions.holistic

# For webcam input:
camera_device = "/dev/depth_cam" 
if camera_device == "/dev/depth_cam":
    os.system('sudo ~/jetauto_ws/src/jetauto_example/scripts/color_detect/reset.sh')
    cap = cv2.VideoCapture(camera_device)
else:
    cap = cv2.VideoCapture("/dev/usb_cam")
print('\n******Press any key to exit!******')
fps = fps.FPS()
with mp_holistic.Holistic(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as holistic:
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
    results = holistic.process(image)

    # Draw landmark annotation on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    mp_drawing.draw_landmarks(
        image,
        results.face_landmarks,
        mp_holistic.FACEMESH_CONTOURS,
        landmark_drawing_spec=None,
        connection_drawing_spec=mp_drawing_styles
        .get_default_face_mesh_contours_style())
    mp_drawing.draw_landmarks(
        image,
        results.pose_landmarks,
        mp_holistic.POSE_CONNECTIONS,
        landmark_drawing_spec=mp_drawing_styles
        .get_default_pose_landmarks_style())
    fps.update()
    result_image = fps.show_fps(cv2.flip(image, 1))
    # Flip the image horizontally for a selfie-view display.
    cv2.imshow('MediaPipe Holistic', result_image)
    key = cv2.waitKey(1)
    if key != -1:
        break
cap.release()
cv2.destroyAllWindows()
