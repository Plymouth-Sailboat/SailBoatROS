[![Current version on ROS](https://img.shields.io/badge/ROS-Melodic-blue.svg)](http://wiki.ros.org/melodic)
![C++ Wrapper](https://img.shields.io/badge/C%2B%2B-100%25-green.svg)
![Python Wrapper](https://img.shields.io/badge/Python-100%25-green.svg)
[![Current version of release](https://img.shields.io/github/release/Plymouth-Sailboat/SailBoatROS/all.svg)](https://github.com/Plymouth-Sailboat/SailBoatROS/releases/latest)
[![Build Status](https://travis-ci.org/Plymouth-Sailboat/SailBoatROS.svg?branch=master)](https://travis-ci.org/Plymouth-Sailboat/SailBoatROS)

# SailBoatROS

C++ and Python code for [Plymouth's Autonomous Sailboat](https://plymouth-sailboat.github.io/). This contains the catkin workspace of the nodes for the sailboat. Every controllers subscribe to topics sent by the arduino and publish commands to it. It is made of classes in C++ and Python for easy integration.  
For a full view of the package [go here](https://github.com/Plymouth-Sailboat/SailBoatROS/wiki/Sailrobot-Package).

An SD Card image is available [here](https://github.com/Plymouth-Sailboat/SailBoatROS/releases/latest) containing a complete environment for the sailboat. You will need a minimum of 8Gb SD card and you can use [Etcher](https://etcher.io/) to write the image directly to your SD Card.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. For a full tutorial for the Raspberry Pi installation follow [Preparing the Raspberry Pi](https://github.com/Plymouth-Sailboat/SailBoatROS/wiki/Preparing-the-Raspberry-Pi) in the wiki.

You can either install the image from the [latest release](https://github.com/Plymouth-Sailboat/SailBoatROS/releases/latest) directly to your SD card or you can follow the guide here or [on the wiki](https://github.com/Plymouth-Sailboat/SailBoatROS/wiki/Preparing-the-Raspberry-Pi) to install from scratch.

### Prerequisites

- [ROS Kinetic](http://wiki.ros.org/kinetic) OR [ROS Melodic](http://wiki.ros.org/melodic)
- Raspberry Pi 3 (B or B+) connected to the Arduino with [Arduino Code Uploaded](https://github.com/Plymouth-Sailboat/SailBoatArduinoInterface)
- [Raspicam Node](https://github.com/UbiquityRobotics/raspicam_node)
- [Rosserial-Arduino Node](http://wiki.ros.org/rosserial_arduino)

**Note :** some nodes use Python3. For this to work with ROS, ROS needs to be installed using python3 dependencies. This is done by executing :
```
sudo apt install python3-pip python3-yaml
pip3 install rospkg catkin_pkg
```

### Installing

Create your catkin workspace if you haven't already and clone the sources in there :

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/plymouth-sailboat/SailBoatROS.git .
```

Make sure you have all the [dependencies](https://github.com/Plymouth-Sailboat/SailBoatROS#dependencies) for the build to work. In summary execute the commands here :

```
cd ~/catkin_ws/
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -y ##installing all the dependencies related to ROS
sudo apt install libnlopt-dev python-numpy python-pygame #installing third-party dependencies
pip install -r requirements.txt #installing python related dependencies
pip3 install -r requirements.txt #installing python3 related dependencies
```

Catkin_make to build the packages in the workspace. Finally source the catkin workspace to have access to the nodes :

```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

**Note : ** You may have to run catkin_make multiple time. This is due mostly because the Raspberry Pi doesn't have enough RAM memory to build everything in one go. Look into adding some swap memory (even if it is not recommended for Raspberry Pi).

### Usage

To launch the communication with the Arduino you have to launch the [rosserial_python](http://wiki.ros.org/rosserial_python) node with the proper USB device, /dev/ttyACM0 in our case :

```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

You can then launch any nodes of the sailboat e.g. :

```
rosrun sailrobot line_following_node
```

Or you can run the prebuilt launch config file which will launch a simulation and visualization of the boat :

```
roslaunch sailrobot simuBoat.launch
```

or you can launch the communication with the Arduino and the rest of the sensors :

```
roslaunch sailrobot start.launch
```

### Dependencies

We use standard messages as much as possible and try to have as less dependencies as possible.

This repo expects to have a working ROS Kinetic/ROS Melodic machine. If not, install it from [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)/[ROS Melodic](http://wiki.ros.org/melodic/Installation).  
If you are on a raspberry pi 3 we suggest installing the [Ubiquity Robotics image](https://downloads.ubiquityrobotics.com/) which already has ROS Kinetic installed.  
If you are on a raspberry pi 3 b+, you can install Ubuntu Mate 18.04 and then install ROS Melodic.

One particular third-party library is used for control optimization. NLOpt. To install it, type :  
```
sudo apt-get install libnlopt-dev
```

#### ROS Dependencies
We use the message [gps_common/GPSFix](http://docs.ros.org/hydro/api/gps_common/html/msg/GPSFix.html) for our GPS, which you will need to install the dependency.
To communicate with the Arduino we use the `rosserial-arduino` [node](http://wiki.ros.org/rosserial_arduino).
Some dependencies are needed for specific modules :
We use cv_bridge to use openCV with ROS. Related to the node Obstacle_avoidance.
We use serial to use the XBee module. Related to the package xbee_serial.

To install them, you can run (when in the catkin workspace `catkin_ws/` :
```
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
```
This will install all the dependencies used by the package.  
Or execute, with [distro] being the ROS distro used (in this case __melodic__) :
```
sudo apt-get install ros-[distro]-gps-common
sudo apt-get install ros-[distro]-visualization-msgs
sudo apt-get install ros-[distro]-rosserial-arduino
sudo apt-get install ros-[distro]-rosserial

sudo apt-get install ros-[distro]-cv-bridge
sudo apt-get install ros-[distro]-serial
```

Because we use the raspberry pi camera, you will need [Raspicam Node](https://github.com/UbiquityRobotics/raspicam_node). There are multiple raspicam_node out there, either version should work. While not all our controls use the camera, you will need this for the complete sailboat to work.

Those are the ROS dependencies used :

```
rospy roscpp std_msgs gps_common
```
#### Python Dependencies
For python controllers, the only dependencies used is [NumPy](http://www.numpy.org/), so you will need to add it to python dependencies :
```
pip install numpy
```
Numpy installation might not work on the Raspberry Pi as expected. In that case, instead of running the above commands, run :  
```
sudo apt-install python-numpy
```

There are other dependencies in the sailrobot_modules package (PyGame, Digi Xbee library, PyUDev). Pygame is better installed using apt rather than pip :
```
sudo apt install python-pygame
```

And now you can run in the catkin workspace (`catkin_ws/`)
```
pip install -r requirements.txt
pip3 install -r requirements.txt
```

**Note**:You will have an error at the end of the pip install,  this is normal.

**In summary execute the commands here :**

```
cd ~/catkin_ws/
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -y ##installing all the dependencies related to ROS
sudo apt install libnlopt-dev python-numpy python-pygame #installing third-party dependencies
pip install -r requirements.txt #installing python related dependencies
pip3 install -r requirements.txt #installing python3 related dependencies
```

## Authors

* [**Ulysse VAUTIER**](https://ulyssevautier.github.io/) - *Main Author*
* [**Christophe VIEL**](https://www.researchgate.net/profile/Christophe_Viel)

## Contributors
* [**Alexandre COURJAUD**](https://github.com/AlexandreCourjaud/Stage2APlymouth)
* [**Corentin JEGAT**](https://github.com/corentin-j/wrsc_plymouth_2019)

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
* [GLM](https://github.com/g-truc/glm) : Math C++ library compatible with OpenGL, using GLSL shader language syntax.

## Look at the Wiki!
If you want more information about the raspberry pi and the boat, please look at [the wiki](https://github.com/Plymouth-Sailboat/SailBoatROS/wiki)
