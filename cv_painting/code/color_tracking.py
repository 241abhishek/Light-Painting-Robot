# import opencv
import cv2
import numpy as np

calibration_file = 'src/Light-Painting-Robot/cv_painting/code/calibration_values.txt'

# read calibration values from an external txt file
def read_calibration_values(filepath): 
    # create two numpy arrays to store the lower and upper bounds
    lower_bound = np.zeros(3)
    upper_bound = np.zeros(3)
    # open the file in read mode
    with open(filepath, 'r') as file:
        # read the lines from the file
        lines = file.readlines()
        # loop through the lines and store the values in the arrays
        for i, line in enumerate(lines):
            if i == 0:
                lower_bound = np.array([int(x) for x in line.split(',')])
            else:
                upper_bound = np.array([int(x) for x in line.split(',')])
    return lower_bound, upper_bound

# display a video file
def display_video(filepath):
    # read in a video file 
    cap = cv2.VideoCapture(filepath)

    # Check if the video file is opened successfully
    if not cap.isOpened():
        print("Error: Unable to open video file.")
        return 

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

# define a calibration routine to select the color to track
# use trackbars to select the color to track
def color_calibration(filepath):
    # read in a video file 
    cap = cv2.VideoCapture(filepath)

    # Check if the video file is opened successfully
    if not cap.isOpened():
        print("Error: Unable to open video file.")
        return 
    
    # read in the calibration values from an external file
    lower_bound, upper_bound = read_calibration_values(calibration_file)

    # Create a window to display the video
    cv2.namedWindow('Video')

    # Create trackbars to select the color to track
    cv2.createTrackbar('Hue Min', 'Video', 0, 179, lambda x: x)
    cv2.createTrackbar('Hue Max', 'Video', 0, 179, lambda x: x)
    cv2.createTrackbar('Sat Min', 'Video', 0, 255, lambda x: x)
    cv2.createTrackbar('Sat Max', 'Video', 0, 255, lambda x: x)
    cv2.createTrackbar('Val Min', 'Video', 0, 255, lambda x: x)
    cv2.createTrackbar('Val Max', 'Video', 0, 255, lambda x: x)

    # Set the trackbar positions to the calibration values
    cv2.setTrackbarPos('Hue Min', 'Video', lower_bound[0])
    cv2.setTrackbarPos('Hue Max', 'Video', upper_bound[0])
    cv2.setTrackbarPos('Sat Min', 'Video', lower_bound[1])
    cv2.setTrackbarPos('Sat Max', 'Video', upper_bound[1])
    cv2.setTrackbarPos('Val Min', 'Video', lower_bound[2])
    cv2.setTrackbarPos('Val Max', 'Video', upper_bound[2])

    # define frame rate variable
    fps = 100

    waitkey_counter = 4
    fps = fps * waitkey_counter

    # Loop through frames and display them
    while True:
        ret, frame = cap.read()
        # resize the video frame
        frame = cv2.resize(frame, (640, 480))

        # Check if frame is read successfully
        if not ret:
            print("Video playback completed.")
            break

        # Convert the frame from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get the current positions of the trackbars
        h_min = cv2.getTrackbarPos('Hue Min', 'Video')
        h_max = cv2.getTrackbarPos('Hue Max', 'Video')
        s_min = cv2.getTrackbarPos('Sat Min', 'Video')
        s_max = cv2.getTrackbarPos('Sat Max', 'Video')
        v_min = cv2.getTrackbarPos('Val Min', 'Video')
        v_max = cv2.getTrackbarPos('Val Max', 'Video')

        # Create a lower and upper bound for the color to track
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])

        # Create a mask to display the color to track
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # overlay the mask on the original frame
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Display the frame
        cv2.imshow('Video', result)

        # Wait for frame rate, and break the loop if 'q' key is pressed
        if cv2.waitKey(int(1000/fps)) & 0xFF == ord('q'):
            break
        
        # if key 's' is pressed, then save the color to track
        if cv2.waitKey(int(1000/fps)) & 0xFF == ord('s'):
            print(f'Color to track: {lower_bound}, {upper_bound}')
            # write the color to track to an external file
            with open(calibration_file, 'w') as file:
                file.write(f'{lower_bound[0]},{lower_bound[1]},{lower_bound[2]}\n')
                file.write(f'{upper_bound[0]},{upper_bound[1]},{upper_bound[2]}')
            break

        # if key 'r' is pressed, then reset the trackbars to the default min-max values
        if cv2.waitKey(int(1000/fps)) & 0xFF == ord('r'):
            cv2.setTrackbarPos('Hue Min', 'Video', 0)
            cv2.setTrackbarPos('Hue Max', 'Video', 179)
            cv2.setTrackbarPos('Sat Min', 'Video', 0)
            cv2.setTrackbarPos('Sat Max', 'Video', 255)
            cv2.setTrackbarPos('Val Min', 'Video', 0)
            cv2.setTrackbarPos('Val Max', 'Video', 255)

        # if key 'f' is pressed, then read the calibration values from the external file and set the trackbars
        if cv2.waitKey(int(1000/fps)) & 0xFF == ord('f'):
            lower_bound, upper_bound = read_calibration_values(calibration_file)
            cv2.setTrackbarPos('Hue Min', 'Video', lower_bound[0])
            cv2.setTrackbarPos('Hue Max', 'Video', upper_bound[0])
            cv2.setTrackbarPos('Sat Min', 'Video', lower_bound[1])
            cv2.setTrackbarPos('Sat Max', 'Video', upper_bound[1])
            cv2.setTrackbarPos('Val Min', 'Video', lower_bound[2])
            cv2.setTrackbarPos('Val Max', 'Video', upper_bound[2])


    # Release the video capture object and close all windows
    cap.release()
    cv2.destroyAllWindows()

# paint a picture using the tracked color
def light_painting(filepath):
    # read in a video file 
    cap = cv2.VideoCapture(filepath)

    # Check if the video file is opened successfully
    if not cap.isOpened():
        print("Error: Unable to open video file.")
        return 
    
    # read in the calibration values from an external file
    lower_bound, upper_bound = read_calibration_values(calibration_file)

    # Initialize an empty canvas
    canvas = np.zeros((480, 640, 3), dtype=np.uint8)

    # define frame rate variable
    fps = 500

    waitkey_counter = 1
    fps = fps * waitkey_counter

    # Loop through frames and display them
    while True:
        ret, frame = cap.read()
        # Check if frame is read successfully
        if not ret:
            print("Video playback completed.")
            break

        # resize the video frame
        frame = cv2.resize(frame, (640, 480))

        # Convert the frame from BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask to display the color to track
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # overlay the mask on the original frame
        color_region = cv2.bitwise_and(frame, frame, mask=mask)

        # Accumulate the moving color regions over frames to create the final image
        canvas = cv2.add(canvas, color_region)

        # Display the original frame
        cv2.imshow('Video', frame)

        # Display the accumulated image
        cv2.imshow('Painting', canvas)

        # Wait for frame rate, and break the loop if 'q' key is pressed
        if cv2.waitKey(int(1000/fps)) & 0xFF == ord('q'):
            break

    # Release the video capture object and close all windows
    cap.release()
    cv2.destroyAllWindows()

def main():
    filepath = '/home/abhi2001/MSR/winter_2024/winter_project/project_code/src/Light-Painting-Robot/cv_painting/test_videos/Smiley.mp4'
    # color_calibration(filepath)
    light_painting(filepath)

if __name__ == "__main__":
    main()