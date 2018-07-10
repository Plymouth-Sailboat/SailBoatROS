[![Current version on ROS](https://img.shields.io/badge/ROS-Kinetic-blue.svg)](http://wiki.ros.org/kinetic)
![C++ Wrapper](https://img.shields.io/badge/C%2B%2B-100%25-green.svg)
![Python Wrapper](https://img.shields.io/badge/Python-100%25-green.svg)
[![Current version of release](https://img.shields.io/badge/Release-v1.2-green.svg)](https://github.com/Plymouth-Sailboat/SailBoatROS/releases/latest)

# SailBoatROS

C++ and Python code for [Plymouth's Autonomous Sailboat](https://plymouth-sailboat.github.io/). This contains the catkin workspace of the nodes for the sailboat. Every controllers subscribe to topics sent by the arduino and publish commands to it. It is made of classes in C++ and Python for easy integration.  
For a full view of the package [go here](/Plymouth-Sailboat/SailBoatROS/wiki/Sailrobot-Package).

An SD Card image is available [here](https://github.com/Plymouth-Sailboat/SailBoatROS/releases/latest) containing a complete environment for the sailboat. You will need a minimum of 8Gb SD card and you can use [Etcher](https://etcher.io/) to write the image directly to your SD Card.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. For a full tutorial for the Raspberry Pi installation follow [Preparing the Raspberry Pi](/Plymouth-Sailboat/SailBoatROS/wiki/Preparing-the-Raspberry-Pi) in the wiki.

You can either install the image from the [latest release](https://github.com/Plymouth-Sailboat/SailBoatROS/releases/latest) directly to your SD card or you can follow the guide here or [on the wiki](/Plymouth-Sailboat/SailBoatROS/wiki/Preparing-the-Raspberry-Pi) to install from scratch.

### Prerequisites

- [ROS Kinetic](http://wiki.ros.org/kinetic)
- Raspberry Pi 3 connected to the Arduino with [Arduino Code Uploaded](https://github.com/Plymouth-Sailboat/SailBoatArduinoInterface)
- [Raspicam Node](https://github.com/UbiquityRobotics/raspicam_node)
- [Rosserial-Arduino Node](http://wiki.ros.org/rosserial_arduino)

### Installing

Clone the project and catkin_make in the workspace. Source the catkin workspace to have access to the nodes.

```
git clone https://github.com/Plymouth-Sailboat/SailBoatROS.git
cd SailBoatROS/catkin_ws
catkin_make
source devel/setup.bash
```

### Usage

To launch the communication with the Arduino you have to launch the [rosserial_python](http://wiki.ros.org/rosserial_python) node with the proper USB device, /dev/ttyACM0 in our case :

```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

You can then launch any nodes of the sailboat e.g. :

```
rosrun sailrobot potential_field_node
```

Or you can run the prebuilt launch config file which will launch the communication with the arduino :

```
roslaunch sailrobot start.launch
```

or if you know the port on which the arduino is connected :

```
roslaunch sailrobot start.launch usb_port:=<port>
```

### Dependencies

We use standard messages as much as possible and try to have as less dependencies as possible.

This repo expects to have a working ROS Kinetic machine. If not, install it from [ROS Kinetic](http://wiki.ros.org/kinetic/Installation). If you are on a raspberry pi 3 we suggest installing the [Ubiquity Robotics image](https://downloads.ubiquityrobotics.com/) which already has ROS Kinetic installed.

#### ROS Dependencies
We use the message [gps_common/GPSFix](http://docs.ros.org/hydro/api/gps_common/html/msg/GPSFix.html) for our GPS, which you will need to install the dependency.
To communicate with the Arduino we use the `rosserial-arduino` [node](http://wiki.ros.org/rosserial_arduino). To install them, execute 
```
sudo apt-get install ros-kinetic-gps-common
sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial
```

Because we use the raspberry pi camera, you will need [Raspicam Node](https://github.com/UbiquityRobotics/raspicam_node). There are multiple raspicam_node out there, either version should work. While not all our controls use the camera, you will need this for the complete sailboat to work.

Those are the ROS dependencies used :

```
rospy roscpp std_msgs gps_common
```
#### Python Dependencies
For python controllers, the only dependencies used is [NumPy](http://www.numpy.org/), so you will need to add it to python dependencies. Don't forget to upgrade pip first :
```
pip install --upgrade pip
pip install numpy
```

## Authors

* **Ulysse VAUTIER** - *Initial work* - [UlysseVautier](https://ulyssevautier.github.io/)
* **Jian WAN** - [Jian Wan](https://www.plymouth.ac.uk/staff/jian-wan)
* **Christophe Viel** - [Christophe Viel](https://www.researchgate.net/profile/Christophe_Viel)

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
* [GLM](https://github.com/g-truc/glm) : Math C++ library compatible with OpenGL, using GLSL shader language syntax.

## Look at the Wiki!
If you want more information about the raspberry pi and the boat, please look at [the wiki](https://github.com/Plymouth-Sailboat/SailBoatROS/wiki)
