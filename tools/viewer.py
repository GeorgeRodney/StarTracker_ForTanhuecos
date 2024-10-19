import pandas as pd
import matplotlib.pyplot as plt
import os
import re
import FrameInfo as fi
from collections import Counter
import time

DET = 0
TRACK = 1

# Now use an absolute path to the file
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
        temp_track = fi.Track(temp[4], temp[2], temp[3])
        scene[int(temp[0])].append_track(temp_track)
        scene[int(temp[0])].set_track_valid(True)


# Visualize SCENE
plt.figure(figsize= (10,10))
plt.xlim(0, 512)
plt.ylim(0, 512)
plt.gca().invert_yaxis()

for frame in range(len(scene)):

    if (scene[frame].frameValid == True):
        
        if (scene[frame].detValid == True):
            for det in range(len(scene[frame].detections)):
                plt.scatter(scene[frame].detections[det].X, scene[frame].detections[det].Y, s=10, c='red')

        if (scene[frame].trackValid == True):
            for track in range(len(scene[frame].tracks)):
                if (scene[frame].tracks[track].status == 1):
                    plt.scatter(scene[frame].tracks[track].X, scene[frame].tracks[track].Y, edgecolor='blue', facecolor='none', s=100, label='Hollow Circle', marker='o')
                elif (scene[frame].tracks[track].status == 2):
                    plt.scatter(scene[frame].tracks[track].X, scene[frame].tracks[track].Y, edgecolor='green', facecolor='none', s=100, label='Hollow Circle', marker='o')

    plt.pause(1)
    plt.cla()
    plt.xlim(0, 512)
    plt.ylim(0, 512)
    plt.gca().invert_yaxis()

print("Done")