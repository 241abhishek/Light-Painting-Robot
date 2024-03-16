# Light-Painting-Robot
This repository contains code and launch files to create light paintings using the [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) robot and the [Adafruit RGB
LED matrix](https://www.adafruit.com/product/5362?gad_source=1&gclid=CjwKCAjw48-vBhBbEiwAzqrZVL83M_J83a_YD0cCXvJ5pjJzY4Eyq7xs4qoPHL9Pc5ig7hY8ce5mwhoCA9MQAvD_BwE).

## Hardware Requirements
A TurtleBot3 Burger robot fitted with an Adafruit LED matrix and multiple retroreflective markers for pose tracking using the Optitrack Motion Capture system.

![Alt text](images/TurtleBot.JPG)

## TurtleBot Software Setup
To SSH into the TurtleBot, run the following command:
```
ssh -oSendEnv=ROS_DOMAIN_ID username@hostname
```
Clone a fork of the LED matrix repository and build it:
```
git clone https://github.com/241abhishek/rpi-rgb-led-matrix.git
cd rpi-rgb-led-matrix/ && make
cd bindings/python/ && make
```
Then, in the root directory for the matrix library
```
sudo apt-get update && sudo apt-get install python3-dev python3-pillow -y
make build-python PYTHON=$(command -v python3)
sudo make install-python PYTHON=$(command -v python3)
```
Navigate to the led_control ROS2 package located at `/rpi-rgb-led-matrix/bindings/python/led_control`. Build it by running:
```
colcon build --symlink-install
```
Run the LED control nodes as root (required for gpio pins access):
```
sudo su
export ROS_DOMAIN_ID=<your_ros_domain_id>
```
The ROS Domain ID must be the same on the TurtleBot and on the system running all the other ROS nodes.

Source the ROS installtion on the TurtleBot when running nodes as root:
```
source /opt/ros/iron/setup.bash
```
Source the workspace (in the root directory of the led_control package) and run the `circle_display` node:
```
source /opt/ros/iron/setup.bash
ros2 run led_control circle_display
```
In a different shell session start the TurtleBot3 control nodes (not required to run as root):
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```
Standard TurtleBot3 setup instructions can be found [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/).
Upon completing all the steps listed above, the TurtleBot3 is fully setup and ready to create Light Paintings.

## Instructions for the Local System
To import and build all dependencies, clone this repository into the `src` directory of your workspace root. Then from the workspace root directory, run the command:
```
vcs import < src/Light-Painting-Robot/painting.repos
colcon build --symlink-install
```
## Optitrack Motion Capture
Source workspace:
```
source install/setup.bash
```
Launch Motive OptiTrack on the Windows System connected to the camera suite and enable streaming in Settings. Then run the following command on your system:
```
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
```
Check that Optitrack configuration works fine and is connected. As the driver node is a lifecycle node, you should transition to activate:
```
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
```
Create a rigid body in the Motive Optitrack Software to track the turtlebot and enable streaming of rigid body data. Confirm that pose data is being streamed over the network using 
`ros2 topic echo /rigid_bodies`.

## Light Painting

Connect a RealSense camera to your system to record a video feed of the robot moving around in the environment.

Launch all the nodes required to create a light-painting using:
```
ros2 launch turtlebot_control control.launch.xml
```
To confirm that the robot is moving as expected, run the following command to specify a pose for the robot:
```
ros2 service call pose custom_interfaces/srv/Pose "{x: 0.5, y: 1.5, theta: 1.57}"
```
If everything is configured correcly, the robot should autonomously move to the specified pose and come to a stop.

### Color Tacking

Run the following command to display a circle on the LED display for calibration.
```
ros2 service call /set_circle custom_interfaces/srv/CircleProp "{x: 32, y: 32, radius: 10, r: 0, g: 0, b: 255}"
```
- `x`: x-coordinate of the circle's center (0-64)
- `y`: y-coordinate of the circle's center (0-64)
- `radius`: radius of the circle
- `r,g,b`: RGB values for the color of the circle

Calibrate the color tracker using:
```
ros2 service call /load_gcode std_srvs/srv/Empty
```

https://private-user-images.githubusercontent.com/72541517/313359935-b081f62b-7d2a-400a-8e04-e08eb0fe8091.webm?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MTA1NDg4NjIsIm5iZiI6MTcxMDU0ODU2MiwicGF0aCI6Ii83MjU0MTUxNy8zMTMzNTk5MzUtYjA4MWY2MmItN2QyYS00MDBhLThlMDQtZTA4ZWIwZmU4MDkxLndlYm0_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjQwMzE2JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI0MDMxNlQwMDIyNDJaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT03NTU3ODZhMWM2ZWU4Mjk1NTM0MWU3MGVmMzVlNzBiZjg4MmMyNmE5OTNjOWZiOTAxOTdhOWVlNTQwYWNiOWQ0JlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZhY3Rvcl9pZD0wJmtleV9pZD0wJnJlcG9faWQ9MCJ9.3vcmT_y3Nf3_q7bBxflx5Nmazj6GDksQgtxqcqxN_gM

While calibration is running, the user can manipulate HSV values to select a color to track:

- Press `r` to reset the trackbars.

- Press `s` to save the calibrated trackbar values.

- Press `f` to read the trackbar values from the `calibration_values.txt` file.

- Press `q` to quit.

The HSV values are saved to an external file and used to create a mask when contructing a Light Painting. 

### Drawing

Load the Gcode file containing the waypoint data. The robot will begin drawing upon executing the following command:
```
ros2 service call /load_gcode std_srvs/srv/Empty
```

The following service call will bring up a 4x4 video feed of the environment
```
ros2 service call /draw_color std_srvs/srv/Empty
```

https://private-user-images.githubusercontent.com/72541517/313359979-c9cfcd82-1eb0-44e8-b9ea-1bec65679fe7.webm?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MTA1NDg5MTcsIm5iZiI6MTcxMDU0ODYxNywicGF0aCI6Ii83MjU0MTUxNy8zMTMzNTk5NzktYzljZmNkODItMWViMC00NGU4LWI5ZWEtMWJlYzY1Njc5ZmU3LndlYm0_WC1BbXotQWxnb3JpdGhtPUFXUzQtSE1BQy1TSEEyNTYmWC1BbXotQ3JlZGVudGlhbD1BS0lBVkNPRFlMU0E1M1BRSzRaQSUyRjIwMjQwMzE2JTJGdXMtZWFzdC0xJTJGczMlMkZhd3M0X3JlcXVlc3QmWC1BbXotRGF0ZT0yMDI0MDMxNlQwMDIzMzdaJlgtQW16LUV4cGlyZXM9MzAwJlgtQW16LVNpZ25hdHVyZT0wMGQyYTFjOTY0YTcwYTEyODMyYzlhNDgzYzkzZGI2ZmVjYWJiNjU2YTMyOWEyMTkyNzYwMmEzZTNlNjY0ZWRlJlgtQW16LVNpZ25lZEhlYWRlcnM9aG9zdCZhY3Rvcl9pZD0wJmtleV9pZD0wJnJlcG9faWQ9MCJ9.VIsJIz-sEj1of9qq8kJCkQLWtN7wkO-L2_6DlOXc2QU

- Top Left - Original Video
- Top Right - Tracked Color
- Bottom Left - Light Painting
- Bottom Right - Overlay of the Video with the Light Painting 

Press `s` to save the Light Painting images once the robot has finished drawing or `q` to quit without saving.