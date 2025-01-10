#!/usr/bin/env python3
import cv2
import time
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO('best.pt')

# Open the video file
video_path = '/home/wijayapratama/robotBiru_ws/src/robotic_sas_auv_ros/scripts/GateSAUVC2.mp4'  # Replace with the path to your video file
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Cannot open video file")
    exit()

# Get video frame dimensions
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_center_x = frame_width // 2
frame_center_y = frame_height // 2

while True:
    start_time = time.time()

    # Capture frame from video
    ret, frame = cap.read()

    if not ret:
        print("End of video reached or cannot read the video file. Exiting ...")
        break

    # Perform object detection using YOLOv8
    results = model(frame)

    # Draw bounding boxes and labels on the frame
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = box.conf[0].tolist()
            cls = box.cls[0].tolist()
            label = model.names[int(cls)]

            # Draw rectangle and label
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # Calculate the center point of the bounding box
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # Draw the center point of the bounding box
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # Display the coordinates of the center point
            cv2.putText(frame, f'({center_x}, {center_y})', (center_x, center_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    # Draw the center point of the frame
    cv2.circle(frame, (frame_center_x, frame_center_y), 5, (255, 0, 0), -1)

    # Display the coordinates of the center point of the frame
    cv2.putText(frame, f'({frame_center_x}, {frame_center_y})', (frame_center_x + 10, frame_center_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # Display the frame
    cv2.imshow('Video Feed', frame)

    # Calculate and display FPS
    fps = 1.0 / (time.time() - start_time)
    print(f"FPS: {fps:.2f}")

    # Wait 1ms to see if a key is pressed, exit on 'q' key press
    if cv2.waitKey(1) == ord('q'):
        break

# Release the video file and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
