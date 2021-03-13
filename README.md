# interiit21_drdo

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

Using nav.py file

```sh
rosrun interiit21_drdo nav.py
```

