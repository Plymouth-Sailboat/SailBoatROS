/**
 * @file waypoint_follower.cpp
 * @brief Controller Class Implementation
 * @author Ulysse Vautier
 * @date 2018-09-05
 */
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

	/// Getting the necessary data for the control
	dvec2 current = dvec2(gpsMsg.latitude, gpsMsg.longitude);
	float wind = windMsg.theta;
	float boatHeading = (Utility::QuaternionToEuler(imuMsg.orientation)).z;

	/// Calculate the distance to the next waypoint, if close, change to the other waypoint
	float dist = Utility::GPSDist(current, waypoints[currentWaypoint]);
	if(dist < 5){
		publishMSG("PArrived at waypoint " + std::to_string(currentWaypoint));
		*tackingStart = waypoints[currentWaypoint];
		currentWaypoint++;
	}
	currentWaypoint %= nbWaypoints;

	/// Check the tacking and change the heading accordingly
	if(tackingStart == NULL){
		tackingStart = new dvec2(current.x,current.y);
	}

	float heading = Utility::GPSBearing(current, waypoints[currentWaypoint]);
	float windNorth = wind + boatHeading;
	bool isTacking = false;
	if(cos(windNorth - heading) + cos(closeHauled) < 0){
		dvec2 line = normalize(waypoints[currentWaypoint] - (*tackingStart));
		dvec2 currentLine = current - *tackingStart;
		float e = line.x*currentLine.y - line.y*currentLine.x;
		if(abs(e) > rmax/2.0)
			q = sign(e);
		heading = windNorth + M_PI - q*closeHauled;
		isTacking = true;
	}

	/// Log message
	std::string message = "PDistance to next waypoint : " + std::to_string(dist) + "\nTries to go to  " + std::to_string(waypoints[currentWaypoint].x) + " " + std::to_string(waypoints[currentWaypoint].y) + "\nWith heading : " + std::to_string(std::fmod(heading*180/M_PI,360.0)) + "\n";
	if(isTacking)
		message += "TACKING\n";
	publishLOG(message);

	/// Send a command of heading to let the Arduino take care of the sail and rudder angle
	cmd.angular.z = heading;
	return cmd;
}
