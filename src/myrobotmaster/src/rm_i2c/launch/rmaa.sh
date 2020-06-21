#!/bin/bash
export LD_LIBRARY_PATH=LD_LIBRARY_PATH:/opt/ros/melodic/lib/
echo source /home/pi/ros/devel/setup.bash
source /opt/ros/melodic/setup.bash
source /home/pi/ros/devel/setup.bash
roslaunch rm_i2c rm_i2c.launch
