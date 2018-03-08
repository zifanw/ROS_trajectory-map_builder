
1. make
把trajectory_yxsk放到 /src下
然后
catkin_make
之后
source devel/setup.bash

2. IP address

change the ip address of the server in /src/wifi.py and /src/wifi_map.py
the port number is reserved for transmission:

port 60000: trajectory 
port 60001: map 

3. start the server 

For V-Slam, 

roslaunch trajectory_yxsk trajectory_vision.launch

For Laser-Slam,

roslaunch trajectory_yxsk trajectory_laser.launch


<<<<<<< HEAD
=======
=======
Structure:
![alt text](/picture/ros_trajectory.bmp)

>>>>>>> 6c2bdbe02813f6890f5a9607f6be61d4007060d1
