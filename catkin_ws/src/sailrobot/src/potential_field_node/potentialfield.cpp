#include "potential_field_node/potentialfield.hpp"
#include "math.h"
#include <fstream>
#include <iostream>
#include <utilities.hpp>
#include <ros/package.h>

using namespace Sailboat;
using namespace glm;

void PotentialField::setup(ros::NodeHandle* n){
	std::string path = ros::package::getPath("sailrobot");

	std::string goalPath = "/data/goalpoint.txt";
	if (n->hasParam("goal"))
		n->getParam("goal",goalPath);

	waypoints = Utility::ReadGPSCoordinates(path + goalPath, nbWaypoints);
	if(waypoints == NULL){
		std::cerr << "Goals empty" << std::endl;
		exit(0);
	}


        std::string obstaclesPath = "/data/obstacles.txt";
        if (n->hasParam("obstacles"))
                n->getParam("obstacles",obstaclesPath);

	obstacles = Utility::ReadGPSCoordinates(path + obstaclesPath, nbObstacles);
	if(obstacles == NULL){
		std::cerr << "Obstacles Coordinates File not Found" << std::endl;
		exit(0);
	}
}


vec2 PotentialField::distanceVector(vec2 dest, vec2 pos){
	double d = Utility::GPSDist(pos, dest);
	double bearing = Utility::GPSBearing(pos, dest);

	return vec2(d*cos(bearing), d*sin(bearing));
}

geometry_msgs::Twist PotentialField::control(){
	if(nbWaypoints < 1)
		return geometry_msgs::Twist();
	vec2 current(gpsMsg.latitude, gpsMsg.longitude);

	geometry_msgs::Twist cmd;
	vec2 heading;

	for(int i = 0; i < nbWaypoints; ++i){
		heading += distanceVector(current, waypoints[i]);
	}
	heading = normalize(heading);

	bool isInObstacle = false;

	for(int i = 0; i < nbObstacles; ++i){
		vec2 res = distanceVector(current,obstacles[i]);

		if(float dist = length2(res) < 10){
			res /= dist*dist;
			heading += res;
			isInObstacle = true;
		}
	}

	cmd.linear.x = (double)heading.x;
	cmd.linear.y = (double)heading.y;
	cmd.linear.z = 0.0;

	if(isInObstacle)
		publishMSG("PAvoiding obstacle");

	publishMSG("PAttracted by : " + std::to_string(waypoints[0].x) + " " + std::to_string(waypoints[0].y));
	return cmd;
}

