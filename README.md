[![Current version on ROS](https://img.shields.io/badge/ROS-Kinetic-blue.svg)](http://wiki.ros.org/kinetic)
![C++ Wrapper](https://img.shields.io/badge/C%2B%2B-100%25-brightgreen.svg)
![Python Wrapper](https://img.shields.io/badge/Python-80%25-green.svg)

# SailBoatROS
C++ and Python code for [Plymouth's Autonomous Sailboat](http://165.227.238.42/). This contains the catkin workspace of the nodes for the sailboat. Every controllers subscribes to topics sent by the arduino and publish commands to it. It is comprised of classes in C++ and Python for easy integration.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

- Ros Kinetic
- Raspberry Pi connected to the Arduino with [Arduino Code Uploaded](https://github.com/Plymouth-Sailboat/SailBoatArduinoInterface)

### Installing

Clone the project and catkin_make in the workspace.

```
catkin_make
source devel/setup.bash
```

### Usage

You should know have access to the nodes of the sailboat :

```
rosrun sailboat potential_field_node
```

Or you can run prebuilt launch config which will launch the communication with the arduino :

```
roslaunch sailboat start.launch
```

## Authors

* **Ulysse VAUTIER** - *Initial work* - [UlysseVautier](https://github.com/UlysseVautier)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

## Look at the Wiki!
If you want more information about the arduino and the boat, please look at [the wiki](https://github.com/Plymouth-Sailboat/SailBoatROS/wiki)
