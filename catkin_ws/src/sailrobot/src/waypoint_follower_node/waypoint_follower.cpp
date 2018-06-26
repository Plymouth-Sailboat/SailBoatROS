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
}

geometry_msgs::Twist WaypointFollower::control(){
	geometry_msgs::Twist cmd;

	vec2 current = vec2(gpsMsg.latitude, gpsMsg.longitude);

	float dist = Utility::GPSDist(current, waypoints[currentWaypoint]);
	if(dist < 5){
		publishMSG("PArrived at waypoint " + std::to_string(currentWaypoint));
		currentWaypoint++;
	}
	currentWaypoint %= nbWaypoints;
	
	std::string message = "PDistance to next waypoint : " + std::to_string(dist) + "\nessaie d'aller a  " + std::to_string(waypoints[currentWaypoint].x) + " " + std::to_string(waypoints[currentWaypoint].y);
	publishMSG(message);
	
	cmd.angular.z = Utility::GPSBearing(current, waypoints[currentWaypoint]);
	return cmd;
}
