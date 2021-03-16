#!/bin/bash

gnome-terminal -- roslaunch interiit21_drdo world1.launch &&
gnome-terminal -- rosrun interiit21_drdo nav.py && 
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console




