# installation guide SLAM REC-HV and dependencies on ROS Noetic and Ubuntu 20.04


## 1. ROS installation and dependencies.
```linux
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update
```

## 2. installation of python 3.8.
```linux
sudo apt update
sudo apt -y upgrade
```
#### If the python version is different than 3.8 do this, otherwise skip this step
```linux
python3 -V
apt install ´python3.8 -y
```
#### Continue here…
```linux
sudo apt install -y python3-pip
sudo apt install build-essential libssl-dev libffi-dev python3-dev
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg rosdep rosinstall_generator rosinstall wstool vcstools catkin_tools
sudo apt-get install python3.8-dev
```



## 3. Virtual environment setup.
```linux
python3.8 -m venv python38_ws
```
#### Activation virtual environment.
```linux
cd python38_ws
source python38_ws/bin/activate
```

## 4. Setup Workspace.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
echo $ROS_PACKAGE_PATH
#### If everything is ok it should show this path “/home/youruser/catkin_ws/src:/opt/ros/noetic/share” ####
```
## 5. Install pip.
```linux
sudo apt update
apt install python3-pip
```
## 6. Install OPENNI.
```linux
sudo apt-get install ros-noetic-openni-camera
sudo apt-get install python-freenect
sudo apt-get update
```


## 7. Install OpenCV
```linux
sudo pip3 install scikit-build
sudo apt-get install python3-opencv
sudo pip3 install opencv-python
#### To test that the opencv was correctly installed do ####
python
import cv2 as cv
print(cv.__version__)
```
## 8. Dependencies.
```linux
cd
pip install rospkg
pip3 install opencv-python
pip3 install matplotlib
pip3 install openpyxl
pip3 install pandas
pip3 install seaborn
pip3 install sklearn
pip3 install open3d-python
git clone https://github.com/eric-wieser/ros_numpy.git
cd ros_numpy/
python setup.py install
cd
cd catkin_ws
cd src
sudo git clone https://github.com/Slamtec/rplidar_ros.git
catkin_make
sudo apt-get install ros-noetic-openni-camera ros-noetic-openni-launch
```
## 9. ORB-SLAM2 installation and dependencies.
```linux
cd
pip3 install pykalman
pip3 install serial
python -m pip install pyserial
sudo apt-get install cmake
sudo apt-get install g++
sudo apt-get install gcc
sudo apt-get install libglew-dev libpython3.8-dev
sudo apt-get install libeigen3-dev
sudo apt-get install -y doxygen libgl1-mesa-dev libglew-dev cmake libboost-all-dev
sudo apt-get update -y
sudo apt-get install -y libsdl-pango-dev
sudo apt-get install ros-___rosdependencia_____-libg2o
sudo apt-get install autoconf
sudo apt-get install wayland-protocols
sudo apt install libopenexr22
sudo apt install openexr
sudo apt install libopenexr-dev
cd catkin_ws
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
cd ORB_SLAM2
chmod +x build.sh 
./build.sh

```
## ORB_SLAM2 common bug fix
### Bug:

fatal error: …/…/config.h: does not have that file or directory:

### Solution:

Since g2o is not fully compiled, go into the ORB_SLAM2 folder and run ./build.sh again


### Bug:

error: ‘usleep was not declared ……

### Solution:

Add .../ORB_SLAM2/include/System.h: #include <unistd.h>

If you also report this error, add it to the file: #include <unistd.h>

(after ./build_ros.sh also has similar question)

### Bug:

![Image text](https://raw.githubusercontent.com/tjuan45/-Installation_guide_2.0/main/error%20cmakefiles.webp)


### Solution:
open LoopClosing.h, find

typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

change to

typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame *const, g2o::Sim3> > > KeyFrameAndPose;





## 11. Creating a catkin Package rdslam for Rec-HV execution.
```linux
cd ~/catkin_ws/src
catkin_create_pkg rdslam std_msgs rospy roscpp
cd
```
#### save and replace what is at this address in the project folder.
https://github.com/MAB1144-Python/SLAM-REC-HV-version-1.0-test/tree/main/rdslam
## 12. Build a catkin workspace and sourcing the setup file in ROS.
```linux
cd ~/catkin_ws
catkin_make
. ~/catkin_ws/devel/setup.bash
```
# Execution Rec_HV
#### now to run the code connect the RPLIDARA1 and Kinect sensors with adapter to USB type A.
#### and execute the following lines on a terminal.
```linux
cd ~/python36_ws
source py36env/bin/activate
cd
cd catkin_ws/src/rdslam/src
sudo chmod 666 /dev/ttyUSB0.
```
#### Launch Rec-HV.
```linux
cd
roslaunch rdslam Rec_HV.launch
```
