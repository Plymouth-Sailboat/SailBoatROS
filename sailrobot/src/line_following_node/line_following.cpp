#include "line_following_node/line_following.hpp"
#include <math.h>
#include <ros/package.h>
#include <utilities.hpp>
#include <glm/glm.hpp>
#include <iostream>
#include <fstream>

using namespace Sailboat;
using namespace glm;

void LineFollowing::setup(ros::NodeHandle* n){
	std::string waypointPath = "data/line_following.txt";
	if(n->hasParam("waypoints"))
		n->getParam("waypoints",waypointPath);

	if(waypoints == NULL){
	waypoints = Utility::ReadGPSCoordinates(waypointPath, nbWaypoints);
		std::cerr << "Waypoints Coordinates File not Found" << std::endl;
		exit(0);
	}
	if(nbWaypoints < 2){
		std::cerr << "Not Enough Coordinates" << std::endl;
		exit(0);
	}
}

geometry_msgs::Twist LineFollowing::control(){
	geometry_msgs::Twist cmd;
	//Retrieve data
	vec2 current = vec2(gpsMsg.latitude, gpsMsg.longitude);
	float wind = windMsg.theta;
	vec3 heading = Utility::QuaternionToEuler(imuMsg.orientation);
	float windNorth = wind + heading.z;
	
	//Parameters
	float psi = M_PI/4.0;
	float ksi = M_PI/3.0;
	
	/// Calculate the distance to the next waypoint, if close, change to the next waypoint
	float dist = Utility::GPSDist(current, waypoints[currentWaypoint]);
	if(dist < 5){
		publishMSG("PArrived at waypoint " + std::to_string(currentWaypoint));
		currentWaypoint++;
	}
	currentWaypoint %= nbWaypoints;

	vec2 ba = waypoints[(currentWaypoint+1)%nbWaypoints]-waypoints[currentWaypoint];
	float normba = length2(ba);
	vec2 bau = ba/normba;

	vec2 ca = current-waypoints[0];
	float e = bau.x*ca.y - bau.y*ca.x;
	
	float phi = atan2(ba.y,ba.x);
	float thetabar = phi - 2*psi/M_PI*atan(e/r);

	//Check For Tacking	
	thetabar = Utility::TackingStrategy(e,phi,windNorth,thetabar,r,psi,ksi);

	//Standard Command for rudder and sail
	vec2 cmdv = Utility::StandardCommand(heading,thetabar, windNorth, M_PI/3.0);
	cmd.angular.x = cmdv.x;
	cmd.angular.y = cmdv.y;

	//LOG
	publishLOG("PLine following Thetabar : " + std::to_string(thetabar) + " Obj : " + std::to_string(waypoints[1].x) + ", " + std::to_string(waypoints[1].y));

	return cmd;
}
