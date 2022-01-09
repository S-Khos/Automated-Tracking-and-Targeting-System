# Description
Automated detection and targeting system built with Nvidia's DIGITS platform on the Jetson embedded computing system.
Uses OpenCV for facial detection and Nvidia's DetectNet Fast R-CNN based object detection library.
The system uses 2 servos for pan and tilt and a CSI camera with a direct feed to a Jetson Nano.
# Purpose
This system was built the purpose to detect and track specified targets through a live video feed for home security purposes.
The system operates on the Jetson Nano using a PWM module to control the pan and tilt servos to change the POV of video feed as the system tracks locked targets. 
