import cv2
import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

############# OPEN DETECTION FRAME OBJECT ########################
detections = []

############# Blob Detector Params ########################
params = cv2.SimpleBlobDetector_Params()

params.filterByArea = True
params.minArea = 1  # Minimum area of blobs (try lowering this)
params.maxArea = 10000  # Maximum area of blobs (adjust as needed)
params.filterByColor = False
params.filterByCircularity = False
params.filterByConvexity = False
params.filterByInertia = False

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)

################ Open video file #########################
video_path = '~/Desktop/projects/AssociatorTest/StarCatalog.mp4'
video_path = os.path.expanduser(video_path)
cap = cv2.VideoCapture(video_path)

# Check if video opened successfully
if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()

frameNum = 0
# Read and display frames from the webcam
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame.")
        break

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cropped_frame = gray_frame[512:1024, 512:1024]

    print(gray_frame.shape)

    # Detect blobs
    keypoints = detector.detect(gray_frame)
    
    print(frameNum)
    for det in keypoints:
        print(det.pt[0])
        print(det.pt[1])
        detections.append((frameNum, det.pt[0], det.pt[1]))
    print('\n')

    # Draw detected blobs as red circles
    output_image = cv2.drawKeypoints(gray_frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Show the image with the blobs
    cv2.imshow("Blob Detection", output_image)
    cv2.waitKey(1)
    frameNum += 1

# Write the detections to a CSV file without commas
output_path = '~/Desktop/projects/AssociatorTest/build/detections_from_video.csv'
output_path = os.path.expanduser(output_path)
with open(output_path, 'w') as f:  # Use .txt to denote no commas
    for detection in detections:
        f.write(f"{detection[0]} {detection[1]} {detection[2]}\n")  # Write as "frame detX detY"


cap.release()
cv2.destroyAllWindows()
