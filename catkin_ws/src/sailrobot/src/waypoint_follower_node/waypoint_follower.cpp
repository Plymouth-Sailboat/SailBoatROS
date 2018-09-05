#include "waypoint_follower_node/waypoint_follower.hpp"
#include "math.h"
#include <ros/package.h>
#include <utilities.hpp>
#include <glm/glm.hpp>
#include <iostream>
#include <fstream>

using namespace Sailboat;
using namespace glm;

void WaypointFollower::setup(ros::NodeHandle* n){
	std::string waypointPath = "data/waypoints.txt";
	if(n->hasParam("waypoints"))
		n->getParam("waypoints", waypointPath);

	waypoints = Utility::ReadGPSCoordinates(waypointPath, nbWaypoints);
	if(waypoints == NULL){
		std::cerr << "Waypoints Coordinates File not Found" << std::endl;
		exit(0);
	}
	currentWaypoint = 0;
	closeHauled = M_PI/3.0;
}

geometry_msgs::Twist WaypointFollower::control(){
	geometry_msgs::Twist cmd;

	vec2 current = vec2(gpsMsg.latitude, gpsMsg.longitude);
	float wind = windMsg.theta;
	float boatHeading = (Utility::QuaternionToEuler(imuMsg.orientation)).z;

	if(tackingStart == NULL){
		tackingStart = new vec2(current.x,current.y);
	}

	float dist = Utility::GPSDist(current, waypoints[currentWaypoint]);
	if(dist < 5){
		publishMSG("PArrived at waypoint " + std::to_string(currentWaypoint));
		*tackingStart = waypoints[currentWaypoint];
		currentWaypoint++;
	}
	currentWaypoint %= nbWaypoints;
	
	float heading = Utility::GPSBearing(current, waypoints[currentWaypoint]);
	//Tacking CHECK
	float windNorth = wind + boatHeading;
	bool isTacking = false;
	if(cos(windNorth - heading) + cos(closeHauled) < 0){
		vec2 line = normalize(waypoints[currentWaypoint] - (*tackingStart));
		vec2 currentLine = current - *tackingStart;
		float e = line.x*currentLine.y - line.y*currentLine.x;
		if(abs(e) > rmax/2.0)
			q = sign(e);
		heading = windNorth + M_PI - q*closeHauled;
		
		isTacking = true;
	}


	std::string message = "PDistance to next waypoint : " + std::to_string(dist) + "\nTries to go to  " + std::to_string(waypoints[currentWaypoint].x) + " " + std::to_string(waypoints[currentWaypoint].y) + "\nWith heading : " + std::to_string(std::fmod(heading*180/M_PI,360.0)) + "\n";
	if(isTacking)
		message += "TACKING\n";
	publishLOG(message);
	
	cmd.angular.z = heading;
	return cmd;
}
