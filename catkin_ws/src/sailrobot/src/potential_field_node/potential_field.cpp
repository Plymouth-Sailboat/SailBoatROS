#include "potential_field_node/potential_field.hpp"
#include <math.h>
#include <fstream>
#include <iostream>
#include <utilities.hpp>
#include <ros/package.h>

using namespace Sailboat;
using namespace glm;

void PotentialField::setup(ros::NodeHandle* n){
	std::string goalPath = "data/goalpoint.txt";
	if (n->hasParam("goal"))
		n->getParam("goal",goalPath);

	waypoints = Utility::ReadGPSCoordinates(goalPath, nbWaypoints);
	if(waypoints == NULL){
		std::cerr << "Goals empty" << std::endl;
		exit(0);
	}


        std::string obstaclesPath = "data/obstacles.txt";
        if (n->hasParam("obstacles"))
                n->getParam("obstacles",obstaclesPath);

	obstacles = Utility::ReadGPSCoordinates(obstaclesPath, nbObstacles);
	if(obstacles == NULL){
		std::cerr << "Obstacles Coordinates File not Found" << std::endl;
		exit(0);
	}

	if(nbWaypoints < 1){
		std::cerr << "No Waypoints" << std::endl;
		exit(0);
	}

	closeHauled = M_PI/3.0;
}


vec2 PotentialField::distanceVector(vec2 pos, vec2 dest){
	double d = Utility::GPSDist(pos, dest);
	double bearing = Utility::GPSBearing(pos, dest);

	return vec2(-d*sin(bearing), d*cos(bearing));
}

geometry_msgs::Twist PotentialField::control(){
	geometry_msgs::Twist cmd;
	vec2 current(gpsMsg.latitude, gpsMsg.longitude);
	float currentHeading = (Utility::QuaternionToEuler(imuMsg.orientation)).z;

	vec2 heading;

	for(int i = 0; i < nbWaypoints; ++i){
		heading += distanceVector(current, waypoints[i]);
	}

	bool isInObstacle = false;

	for(int i = 0; i < nbObstacles; ++i){
		vec2 res = distanceVector(current,obstacles[i]);

		if(float dist = length(res) < 10){
			res /= dist*dist;
			heading += res;
			isInObstacle = true;
		}
	}

	vec2 windPotential;
	for(int i = -3; i < 3; ++i){
		for(int j = -3; j < 3; ++j){
			float pangle = cos(atan2(j,i)+M_PI/2.0;
			if(cos(pangle-windMsg.theta) < cos(closeHauled)){
				windPotential -= length(vec2(i,j))*vec2(-sin(pangle - currentHeading), cos(pangle - currentHeading)); 
			}
		}
	}
	heading += windPotential;

	

	cmd.linear.x = (double)heading.x;
	cmd.linear.y = (double)heading.y;
	cmd.linear.z = 0.0;

	std::string message = "";

	if(isInObstacle)
		message += "PAvoiding obstacle \n";

	publishMSG(message + "PAttracted by : " + std::to_string(waypoints[0].x) + " " + std::to_string(waypoints[0].y) + " heading of : (" + std::to_string(heading.x) + "," + std::to_string(heading.y) + ")");
	return cmd;
}

