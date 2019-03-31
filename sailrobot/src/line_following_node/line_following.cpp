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

	waypoints = Utility::ReadGPSCoordinates(waypointPath, nbWaypoints);
	if(waypoints == NULL){
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
	vec3 currentXYZ = Utility::GPSToCartesian(current.x, current.y);
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

	vec3 a = Utility::GPSToCartesian(waypoints[(currentWaypoint+1)%nbWaypoints]);
	vec3 b = Utility::GPSToCartesian(waypoints[currentWaypoint]);

	glm::vec3 n = glm::cross(a,b)/(glm::length(a)*glm::length(b));

	float e = glm::dot(currentXYZ,n);
	mat3x2 M;
	M[0][0]=-sin(current.y);
	M[1][0]=cos(current.y);
	M[2][0]=0;
	M[0][1]=-cos(current.y)*sin(current.x);
	M[1][1]=-sin(current.x)*sin(current.y);
	M[2][1]=cos(current.x);

	vec2 ba = M*(b-a);
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
