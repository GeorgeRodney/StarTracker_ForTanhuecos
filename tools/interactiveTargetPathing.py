import pygame
import csv
import random

# Initialize pygame
pygame.init()

# Constants
MEAS_NOISE = 0.9
WINDOW_SIZE = 1024
FPS = 60
BACKGROUND_COLOR = (30, 30, 30)
PATH_COLOR = (200, 0, 0)
DETECTION_COLOR = (0, 255, 255)
MAX_FALSE_ALARMS_PER_FRAME = 0  # Maximum number of false alarms per frame
SAMPLING_DENSITY = 10  # Number of interpolated points between path points

# Set up the display
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption("Path and Target Simulation")
clock = pygame.time.Clock()

# Variables to store paths and targets
paths = []  # List of paths, each path is a list of points [(x1, y1), (x2, y2), ...]
current_path = []

# Function to interpolate between two points
def interpolate_points(p1, p2, num_samples):
    """Interpolate points between p1 and p2 for smoother path."""
    x1, y1 = p1
    x2, y2 = p2
    interpolated_points = []
    
    for i in range(num_samples):
        alpha = i / float(num_samples)
        x = x1 + alpha * (x2 - x1)
        y = y1 + alpha * (y2 - y1)
        interpolated_points.append((x, y))
        
    return interpolated_points

# Function to process paths into frame-based detections with truth IDs
def process_paths_to_detections(paths):
    frame_detections = []
    max_path_length = max(len(path) for path in paths) if paths else 0

    for frame in range(max_path_length):
        for truth_id, path in enumerate(paths):
            if frame < len(path):  # If the frame index exists in the path
                x, y = path[frame]
                frame_detections.append((frame, x + random.uniform(-MEAS_NOISE, MEAS_NOISE), y + random.uniform(MEAS_NOISE, MEAS_NOISE), truth_id))
    
    return frame_detections

# Function to generate false alarms for each frame with truth_id -1
def generate_false_alarms_for_frame(frame):
    num_false_alarms = random.randint(0, MAX_FALSE_ALARMS_PER_FRAME)  # Random number of false alarms per frame
    false_alarms = []
    for _ in range(num_false_alarms):
        # Generate random float values for false alarms
        x = random.uniform(0, WINDOW_SIZE - 1)
        y = random.uniform(0, WINDOW_SIZE - 1)
        false_alarms.append((frame, x, y, -1))  # Add false alarm with truth_id -1
    return false_alarms

# Main loop
running = True
drawing = False

while running:
    screen.fill(BACKGROUND_COLOR)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False  # Exit the loop and close the window
        
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left mouse button
                drawing = True
                current_path = []
        
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:  # Left mouse button
                drawing = False
                if current_path:
                    # Interpolate points between the last two points of the current path
                    if len(current_path) > 1:
                        # Get the last two points to interpolate between
                        interpolated_points = interpolate_points(current_path[-2], current_path[-1], SAMPLING_DENSITY)
                        current_path = current_path[:-1] + interpolated_points  # Replace last segment with interpolated points
                    paths.append(current_path)

        elif event.type == pygame.MOUSEMOTION:
            if drawing:
                current_path.append((event.pos[0] + random.uniform(-MEAS_NOISE, MEAS_NOISE), event.pos[1] + random.uniform(-MEAS_NOISE, MEAS_NOISE)))  # Add sub-pixel variation
    
    # Draw paths with sub-pixel accuracy by rendering as circles
    for path in paths:
        if len(path) > 1:
            for i in range(len(path) - 1):
                pygame.draw.aaline(screen, PATH_COLOR, path[i], path[i+1])  # Use anti-aliased line for smoother paths
    if len(current_path) > 1:
        for i in range(len(current_path) - 1):
            pygame.draw.aaline(screen, PATH_COLOR, current_path[i], current_path[i+1])

    pygame.display.flip()
    clock.tick(FPS)
# Get the maximum number of points across all paths after interpolation
max_frames = max(len(path) for path in paths) if paths else 0

# Process paths to generate frame-based detections with truth IDs
frame_based_detections = process_paths_to_detections(paths)

# Generate false alarms for each frame
all_detections = []
for frame in range(max_frames):
    # Add path detections for this frame
    frame_detections = [det for det in frame_based_detections if det[0] == frame]
    
    # Add false alarms for this frame
    false_alarms = generate_false_alarms_for_frame(frame)
    
    # Combine both detections and false alarms
    all_detections.extend(frame_detections)
    all_detections.extend(false_alarms)

# Save detections to a CSV file
with open('../build/detections.csv', 'w', newline='') as file:
    writer = csv.writer(file, delimiter=' ')
    for detection in all_detections:
        writer.writerow(detection)

pygame.quit()
