#include "potential_field_node/potentialfield.hpp"
#include "math.h"
#include <fstream>
#include <iostream>
#include <utilities.hpp>
#include <ros/package.h>

using namespace Sailboat;

void PotentialField::setup(ros::NodeHandle* n){
	std::string path = ros::package::getPath("sailrobot");
	std::ifstream f(path + "/data/waypoints.txt");
    if(f.good())
		waypoints = Utility::ReadGPSCoordinates(path + "/data/waypoints.txt");
	else{
		std::cerr << "Waypoints Coordinates File not Found" << std::endl;
		exit(0);
	}
	f.close();
	f.open(path + "/data/obstacles.txt");
    if(f.good())
		obstacles = Utility::ReadGPSCoordinates(path + "/data/obstacles.txt");
	else{
		std::cerr << "Obstacles Coordinates File not Found" << std::endl;
		exit(0);
	}
	f.close();
}


tf::Vector3 PotentialField::distanceVector(tf::Vector3 dest, tf::Vector3 pos){
	double d = Utility::GPSDist(pos.getX(), pos.getY(), dest.getX(), dest.getY());
	double bearing = Utility::GPSBearing(pos.getX(), pos.getY(), dest.getX(), dest.getY());

	return tf::Vector3(d*cos(bearing), d*sin(bearing), 0);
}

geometry_msgs::Twist PotentialField::control(){
	int rowsWaypoints =  sizeof(waypoints) / sizeof(waypoints[0]);
	int rowsObstacles =  sizeof(obstacles) / sizeof(obstacles[0]);
	
	if(rowsWaypoints < 1)
		return geometry_msgs::Twist();
	tf::Vector3 current(gpsMsg.latitude, gpsMsg.longitude, 0);

	geometry_msgs::Twist cmd;
	tf::Vector3 heading;

	for(int i = 0; i < rowsWaypoints; ++i){
		heading += distanceVector(current, tf::Vector3(waypoints[i][0], waypoints[i][1], 0));
	}
	heading.normalize();

	for(int i = 0; i < rowsObstacles; ++i){
		tf::Vector3 res = distanceVector(current,tf::Vector3(obstacles[i][0], obstacles[i][1], 0));

		if(float dist = res.length2() < 100){
			res /= dist*dist;
			heading += res;
		}
	}

	cmd.linear.x = (double)heading.getX();
	cmd.linear.y = (double)heading.getY();
	cmd.linear.z = (double)heading.getZ();
	return cmd;
}
