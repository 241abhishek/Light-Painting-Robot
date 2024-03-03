# import opencv
import cv2
import numpy as np

# read in a video file 
filepath = 'src/Light-Painting-Robot/cv_painting/test_videos/Smiley.mp4'
cap = cv2.VideoCapture(filepath)

# Check if the video file is opened successfully
if not cap.isOpened():
    print("Error: Unable to open video file.")

# Loop through frames and display them
while True:
    ret, frame = cap.read()

    # Check if frame is read successfully
    if not ret:
        print("Video playback completed.")
        break

    # Display the frame
    cv2.imshow('Video', frame)

    # Wait for 25 milliseconds, and break the loop if 'q' key is pressed
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()
