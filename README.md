interiit21_drdo

First follow the instructions in [Installation.md](Installation.md) file.

Install the following:

```sh
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```

Then, initialize the catkin workspace:

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Then clone this repo in catkin workspace

```sh
cd ~/catkin_ws/src
git clone https://github.com/vstark21/interiit21_drdo.git
```

and then

```sh
cd ~/catkin_ws
catkin build
```

And make all the files in scripts directory executable

```sh
cd ~/catkin_ws/src/interiit21_drdo/scripts
sudo chmod +x nav.py
```

Now remove `gimbal_small_2d/` directory

```sh
cd ~/.gazebo/models
rm -r gimbal_small_2d
```

Using nav.py file

```sh
rosrun interiit21_drdo nav.py
```


launching rtabmap
```sh
roslaunch interiit21_drdo slam.launch

### or 

roslaunch rtabmap_ros rtabmap.launch     rtabmap_args:="--delete_db_on_start"    frame_id:=camera_link_optical rgb_topic:=/depth_camera/rgb/image_raw     depth_topic:=/depth_camera/depth/image_raw     camera_info_topic:=/depth_camera/depth/camera_info  rviz:=true
```

controlling using setpoint
```bash
rosrun interiit21_drdo nav.py

## along side with this in another terminal

rosrun interiit21_drdo aruco.py
```

## Topics

Octomap output
```bash
/rtabmap/octomap_full		## contains octomap with color
/rtabmap/octomap_binary		## contains octomap with only binary value
```

Odometry
```bash
/mavros/local_position/odom	## Accurate but is slow at 5 Hz
/rtabmap/odom			## Faster method to achive odometry
```

Controlling with setpoint
```bash
/setpoint_array         ## pass a array of Poses with time stamp
```
This topic will be active after launching nav.py
and custom message is used 
```bash
$ rosmsg info interiit21_drdo/Setpoints
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose[] setpoints
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```