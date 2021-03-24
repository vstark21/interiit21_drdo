#!/bin/bash

gnome-terminal -- roslaunch interiit21 interiit_world3.launch &&
gnome-terminal -- roslaunch rtabmap_ros rtabmap.launch     rtabmap_args:="--delete_db_on_start"    frame_id:=camera_link_optical rgb_topic:=/depth_camera/rgb/image_raw     depth_topic:=/depth_camera/depth/image_raw     camera_info_topic:=/depth_camera/depth/camera_info  rviz:=true &&
gnome-terminal -- rosrun interiit21_drdo nav.py && 
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
