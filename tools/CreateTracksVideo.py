import pandas as pd
import matplotlib.pyplot as plt
import os
import cv2
import re
import FrameInfo as fi
from collections import Counter
import numpy as np
import matplotlib.animation as animation
import subprocess

DET = 0
TRACK = 1

######################### RUN TFM #################################
# run_command = "cd ../build && ./run 2>&1 | tee ../output/test.csv"
# subprocess.run(run_command, shell=True, check=True)

############### RUN OUTPUT #####################################
csv_path = '~/Desktop/projects/AssociatorTest/output/test.csv'
csv_path = os.path.expanduser(csv_path)
first_value_counter = Counter(line.split(',')[0] for line in open(csv_path) if line.strip())

# frames = [frameStruct() for _ in range(len(first_value_counter))]
scene = [fi.FrameInfo() for _ in range(len(first_value_counter))]

if os.path.exists(csv_path):
    print("File found!")
else:
    print("File not found.")

# How many lines of the csv files
with open(csv_path, 'r') as file:
    lines = file.readlines()

pattern = r'(-?\d+(?:\.\d+)?)'

# Load line data into structures
for line in range(len(lines)):

    # temp DET   LINE: frame, ID, X, Y
    # temp TRACK LINE: frame, ID, X, Y, Status
    temp = re.findall(pattern, lines[line])
    temp = [float(num) for num in temp]
    scene[int(temp[0])].set_frame_valid(True)

    if (int(temp[1]) == DET):
        temp_det = fi.Detection(temp[2], temp[3])
        scene[int(temp[0])].append_detection(temp_det)
        scene[int(temp[0])].set_det_valid(True)

    if (int(temp[1]) == TRACK):
        temp_track = fi.Track(temp[4], temp[2], temp[3], temp[5], temp[6])
        scene[int(temp[0])].append_track(temp_track)
        scene[int(temp[0])].set_track_valid(True)

# Visualize SCENE
# Create a figure and axis for the animation
fig = plt.figure(figsize=(20, 10))
ax = plt.gca()
ax.set_facecolor('black')
plt.xlim(0, 1920)
plt.ylim(0, 1080)
plt.gca().invert_yaxis()

###############################################################
# converged_track_positions = []

# Update function to plot detections and tracks for each frame
def update(frame):
    ax.cla()  # Clear the axis for the next frame
    ax.set_facecolor('black')
    plt.xlim(0, 1920)
    plt.ylim(0, 1080)
    plt.gca().invert_yaxis()

    # if converged_track_positions:
    #     for pos in converged_track_positions:
    #         plt.scatter(pos[0], pos[1], s=1, color='green')

    if scene[frame].frameValid:
        # Plot detections if valid
        if scene[frame].detValid:
            for det in scene[frame].detections:
                plt.scatter(det.X, det.Y, s=1, label='DETECTION', marker='o', c='white')
        
        # Plot tracks if valid
        if scene[frame].trackValid:
            for track in scene[frame].tracks:
                if track.status == 1:
                    plt.scatter(track.X, track.Y, edgecolor='blue', facecolor='none', s=100, label='OPEN TRACK', marker='o', linewidths=2)
                elif track.status == 2:
                    plt.scatter(track.X, track.Y, edgecolor='green', facecolor='none', s=200, label='CONVERGED TRACK', marker='^', linewidths=2)
                    # converged_track_positions.append((track.X, track.Y))
    return ax,

# Create the animation using FuncAnimation
ani = animation.FuncAnimation(fig, update, frames=len(scene), interval=50, blit=False)

# Optionally, save the animation as a .avi file
vid_output_path = '~/Desktop/projects/AssociatorTest/output/TracksAccelerationStarsV1.avi'
vid_output_path = os.path.expanduser(vid_output_path)
ani.save(vid_output_path, writer='ffmpeg', fps=30)

plt.show()