[![Current version on ROS](https://img.shields.io/badge/ROS-Kinetic-blue.svg)](http://wiki.ros.org/kinetic)
![C++ Wrapper](https://img.shields.io/badge/C%2B%2B-100%25-brightgreen.svg)
![Python Wrapper](https://img.shields.io/badge/Python-80%25-green.svg)

# SailBoatROS

C++ and Python code for [Plymouth's Autonomous Sailboat](http://165.227.238.42/). This contains the catkin workspace of the nodes for the sailboat. Every controllers subscribe to topics sent by the arduino and publish commands to it. It is comprised of classes in C++ and Python for easy integration.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

- ROS Kinetic
- Raspberry Pi connected to the Arduino with [Arduino Code Uploaded](https://github.com/Plymouth-Sailboat/SailBoatArduinoInterface)
- [Raspicam Node](https://github.com/UbiquityRobotics/raspicam_node)
- [Rosserial-Arduino Node](http://wiki.ros.org/rosserial_arduino)

### Dependencies

This repo expects to have a working ROS Kinetic machine. If not, install it from [ROS Kinetic](http://wiki.ros.org/kinetic/Installation).
We use standard messages as much as possible and try to have as less dependencies as possible.

To communicate with the Arduino we use the `rosserial-arduino` [node](http://wiki.ros.org/rosserial_arduino). To install it, execute 
```
sudo apt-get install ros-indigo-rosserial-arduino
sudo apt-get install ros-indigo-rosserial
```

You will need the basic dependency of a ROS package. If you followed the installation instructions or the [ROS Tutorial](http://wiki.ros.org/ROS/Tutorials), you shouldn't have anything else to install. Just to list the ROS dependencies used :

```
rospy roscpp std_msgs gps_common
```

Because we use the raspberry pi camera, you will need [Raspicam Node](https://github.com/UbiquityRobotics/raspicam_node). While not all controls use the camera, you will need this for the complete sailboat to work.

### Installing

Clone the project and catkin_make in the workspace. Source the catkin workspace to have access to the nodes.

```
catkin_make
source devel/setup.bash
```

### Usage

You should now have access to the nodes of the sailboat :

```
rosrun sailboat potential_field_node
```

Or you can run the prebuilt launch config file which will launch the communication with the arduino :

```
roslaunch sailboat start.launch
```

## Authors

* **Ulysse VAUTIER** - *Initial work* - [UlysseVautier](https://github.com/UlysseVautier)
* **Jian WAN** - [Jian Wan](https://www.plymouth.ac.uk/staff/jian-wan)
* **Christophe Viel** - [Christophe Viel](https://www.researchgate.net/profile/Christophe_Viel)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

## Look at the Wiki!
If you want more information about the raspberry pi and the boat, please look at [the wiki](https://github.com/Plymouth-Sailboat/SailBoatROS/wiki)
