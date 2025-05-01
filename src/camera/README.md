# detection
 Ros Node:
color detection and angle 
this code is detecting color and finding its centroid of an object
then by using formula:

***Theta = ((centroid of x - width of the frame/2)/(width of the frame/2))*(Field Of View/2)***

(check FOV of your camera it might differ, but usually webcameras have Field Of View between 60-70 degrees)

left =-theta,
right= theta,
center=0

To connect Arduino with ROS:

**we need to change port in serial_bridge.py (eg.COM8)**. Python serial should be installed in your ROS.
After installation and running the python code, arduino script and python should start to work
(they both will use same port. In case if you want to change the arduino code you have to stop running python)

_open terminal 1:_

> run serial_bridge.py

_open new terminal:_

> run the node (as a default it will capture yellow object).
 It should open camera and send angles (try to move left and right)


in case if you want to change _target_color_ run:

> ros2 param set /color_detection target_color <name>


well done :)
now it should open the camera and mask with angles in the terminal
