import pandas as pd
import matplotlib.pyplot as plt
import os

# Now use an absolute path to the file
csv_path = '~/Desktop/projects/AssociatorTest/output/test.csv'  # Replace with the actual absolute path

if os.path.exists(csv_path):
    print("File found!")
else:
    print("File not found.")

# Load your data
col_names = ['frame', 'ID', 'x', 'y']
data = pd.read_csv(csv_path, names=col_names)  # Replace with your actual file

print(data.columns)

# Assuming your CSV has columns like: frame, det_x, det_y, est_x, est_y, pred_x, pred_y
frames = data['frame'].unique()

plt.figure(figsize=(8,8))

# Loop through each frame to plot
for frame in frames:
    frame_data = data[data['frame'] == frame]
    print(frame_data)
    
    # Plot detections
    plt.scatter(frame_data['x'], frame_data['y'], c='blue', label='Detection', alpha=0.5)

    # # Plot estimated positions
    # plt.scatter(frame_data['est_x'], frame_data['est_y'], c='green', label='Estimated', marker='x', alpha=0.7)
    
    # # Plot predicted positions
    # plt.scatter(frame_data['pred_x'], frame_data['pred_y'], c='red', label='Predicted', marker='o', alpha=0.7)

    # Add titles and labels
    plt.title(f'Tracking Frame {frame}')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    
    # Set x and y limits
    plt.xlim([0, 511])
    plt.ylim([0, 511])

    # Add legend
    # plt.legend()

    # Show the frame
    plt.show(block=False)  # Use block=False to not block the code execution
    plt.pause(0.1)  # Pause for 1 second

    # Clear figure for the next frame
    plt.clf()  # Clear the current figure

print("Done")