#include "line_following_node/line_following.hpp"
#include "math.h"
#include <ros/package.h>
#include <utilities.hpp>
#include <glm/glm.hpp>
#include <iostream>
#include <fstream>

using namespace Sailboat;
using namespace glm;

void LineFollowing::setup(ros::NodeHandle* n){
	int nbWaypoints = 0;

	std::string path = ros::package::getPath("sailrobot");

	std::string waypointPath = "/data/line_following.txt";
	if(n->hasParam("waypoints"))
		n->getParam("waypoints",waypointPath);

	waypoints = Utility::ReadGPSCoordinates(path + waypointPath, nbWaypoints);
	if(waypoints != NULL){
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
	
	vec2 current = vec2(gpsMsg.latitude, gpsMsg.longitude);
	float wind = windMsg.theta;
	vec3 heading = Utility::QuaternionToEuler(imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w);
	
	int q = 0;
	float r = 50.0;
	float psi = M_PI/4.0;
	float ksi = M_PI/3.0;
	vec2 ba = waypoints[1]-waypoints[0];
	float normba = length2(ba);
	
	vec2 bau = ba/normba;
	vec2 ca = current-waypoints[0];
	float e = bau.x*ca.y - bau.y*ca.x;
	if(abs(e) > r/2)
		q = e>=0?1:-1;
	float phi = atan2(ba.y,ba.x);
	float theta = phi - 2*psi/M_PI*atan(e/r);
	
	float thetabar = 0;
	
	if(cos(wind-theta)+cos(ksi) < 0 || (abs(e) < r && (cos(wind-theta)+cos(ksi) < 0)))
		thetabar = M_PI + wind - q*ksi;
	else
		thetabar = theta;
	
	if(cos(heading.z - thetabar) >= 0)
		cmd.angular.x = 45.0*sin(heading.z-thetabar);
	else
		cmd.angular.x = 45.0*((sin(heading.z-thetabar)>=0)?1:-1);
	cmd.angular.y = M_PI/2.0*((wind-thetabar)+1)/2.0;

	publishMSG("PLine following Thetabar : " + std::to_string(thetabar) + " Obj : " + std::to_string(waypoints[1].x) + ", " + std::to_string(waypoints[1].y));
	
	return cmd;
}
