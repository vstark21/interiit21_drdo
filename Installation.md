# Installation 
## ROS
You can find these installation instructions [here](wiki.ros.org/melodic/Installation/Ubuntu).
#### Setup your sources.list
```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
#### Set up your keys
```sh
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
#### Update packages and install ROS
```sh
sudo apt update
sudo apt install ros-melodic-desktop-full
```
#### Setup the environment
```sh
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc	
```
#### Dependencies
```sh
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```
#### Rosdep
```sh
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

## Ardupilot
### Installing Ardupilot and MAVProxy
#### Clone ArduPilot

In home directory:
```sh
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout Copter-3.6
git submodule update --init --recursive
```

#### Install dependencies:
```sh
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect
```

#### Use pip (Python package installer) to install mavproxy:
```sh
sudo pip install future pymavlink MAVProxy
```

Open `~/.bashrc` for editing:
```sh
gedit ~/.bashrc
```

Add these lines to end of `~/.bashrc` (the file open in the text editor):
```sh
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```

Save and close the text editor.

Reload `~/.bashrc`:
```sh
. ~/.bashrc
```

Run SITL (Software In The Loop) once to set params:
```sh
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```
## Gazebo and Plugins
#### Gazebo

Setup your computer to accept software from http://packages.osrfoundation.org:
```sh
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:
```sh
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Reload software list:
```sh
sudo apt update
```
Install Gazebo:
```sh
sudo apt install gazebo9 libgazebo9-dev
```
### Install Gazebo plugin for APM (ArduPilot Master) :
```sh
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
git checkout dev
```
build and install plugin
```sh
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
```sh
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```
Set paths for models:
```sh
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

#### Run Simulator
In one Terminal (Terminal 1), run Gazebo:
```sh
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

In another Terminal (Terminal 2), run SITL:
```sh
cd ~/ardupilot/ArduCopter/
sim_vehicle.py -v ArduCopter -f gazebo-iris --console
```

In another Terminal (Terminal 3), run Navigation Stack:
```sh
rosrun interiit21_drdo nav.py 
```

Then in the MavProxy Console
```bash
STABILIZE> arm throttle
STABILIZE> mode GUIDED
```

This should enable the nav.py console to control the drone manually.

