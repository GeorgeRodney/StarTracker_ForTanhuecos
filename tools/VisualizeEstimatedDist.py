import pandas as pd
import matplotlib.pyplot as plt
import os
import cv2
import re
import FrameInfo as fi
from FrameInfo import Track
from collections import Counter
import numpy as np
import matplotlib.animation as animation
import subprocess

DET = 0
TRACK = 1

############### RUN OUTPUT #####################################
csv_path = '~/Desktop/projects/AssociatorTest/output/test.csv'
csv_path = os.path.expanduser(csv_path)
first_value_counter = Counter(line.split(',')[0] for line in open(csv_path) if line.strip())

scene = [fi.FrameInfo() for _ in range(len(first_value_counter))]

if os.path.exists(csv_path):
    print("File found!")
else:
    print("File not found.")

# How many lines of the csv files
with open(csv_path, 'r') as file:
    lines = file.readlines()

ROW = 1920
COL = 1080
bits = 1024
# Define the parameters of the Gaussian
x = np.linspace(0, ROW, bits)
y = np.linspace(0, COL, bits)
X, Y = np.meshgrid(x, y)


pattern = r'(-?\d+(?:\.\d+)?)'

# Visualize SCENE
# Create a figure and axis for the animation
fig = plt.figure(figsize=(10, 10))
ax = plt.gca()

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

mu = (0, 0)
P = np.array([[0.1, 0], [0, 0.1]])
totalDist = fi.calculate_gaussian_2d(mu, P, bits, COL, ROW)

# Update function to plot detections and tracks for each frame
def update(frame):
    # ax.cla()  # Clear the axis for the next frame
    mu = (0, 0)
    P = np.array([[0.1, 0], [0, 0.1]])
    totalDist = fi.calculate_gaussian_2d(mu, P, bits, COL, ROW)
    VIS_SCALE = 100.0

    if scene[frame].frameValid:
        for track in range(len(scene[frame].tracks)):
            print(f"Frame {frame}, Track {track}: P00={scene[frame].tracks[track].P00}, P11={scene[frame].tracks[track].P11}")
            mu = (scene[frame].tracks[track].X, scene[frame].tracks[track].Y)
            P = np.array([[scene[frame].tracks[track].P00*VIS_SCALE, 0], [0, VIS_SCALE*scene[frame].tracks[track].P11]])
            
            if ( (P[0][0] != 0.0) & (P[1][1] != 0.0) ):
                distribution = fi.calculate_gaussian_2d(mu, P, bits, COL, ROW)
                totalDist += distribution
            
    plt.contourf(X, Y, totalDist, cmap='viridis')
    # plt.gca().invert_yaxis()  # Flip the y-axis
    totalDist = np.zeros_like(distribution)

    return ax,

# Create the animation using FuncAnimation
ani = animation.FuncAnimation(fig, update, frames=len(scene), interval=50, blit=False)

# Optionally, save the animation as a .avi file
vid_output_path = '~/Desktop/projects/AssociatorTest/output/TrackerDistributionAccelerationV1.mp4'
vid_output_path = os.path.expanduser(vid_output_path)
ani.save(vid_output_path, writer='ffmpeg', fps=30)

plt.show()


