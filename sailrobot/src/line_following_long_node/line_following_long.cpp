#include "line_following_long_node/line_following_long.hpp"
#include <math.h>
#include <ros/package.h>
#include <utilities.hpp>
#include <glm/glm.hpp>
#include <iostream>
#include <fstream>

using namespace Sailboat;
using namespace glm;

void LineFollowingLong::setup(ros::NodeHandle* n){
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

geometry_msgs::Twist LineFollowingLong::control(){
	geometry_msgs::Twist cmd;
	//Retrieve data
	vec2 current = vec2(gpsMsg.latitude, gpsMsg.longitude);
	vec2 currentRad = vec2(gpsMsg.latitude*M_PI/180.0, gpsMsg.longitude*M_PI/180.0);
	vec3 currentXYZ = Utility::GPSToCartesian(current);
	vec3 heading = Utility::QuaternionToEuler(imuMsg.orientation);
	float windNorth = Utility::RelativeToTrueWind(vec2(velMsg.linear.x,velMsg.linear.y),heading.z,windMsg.theta,windMsg.x);
	
	//Parameters
	float psi = M_PI/4.0;
	float ksi = M_PI/3.0;
	
	/// Calculate the distance to the next waypoint, if close, change to the next waypoint
	float dist = Utility::GPSDist(current, waypoints[(currentWaypoint+1)%nbWaypoints]);
	if(dist < 5){
		publishMSG("PArrived at waypoint " + std::to_string(currentWaypoint));
		currentWaypoint++;
	}
	currentWaypoint %= nbWaypoints;

	vec3 b = Utility::GPSToCartesian(waypoints[(currentWaypoint+1)%nbWaypoints]);
	b = b/glm::length(b);
	vec3 a = Utility::GPSToCartesian(waypoints[currentWaypoint]);
	a = a/glm::length(a);
	vec3 n = glm::cross(a,b);
	float e = glm::dot(currentXYZ,n);
	mat3x2 M;
	M[0][0]=-sin(currentRad.y);
	M[1][0]=cos(currentRad.y);
	M[2][0]=0;
	M[0][1]=-cos(currentRad.y)*sin(currentRad.x);
	M[1][1]=-sin(currentRad.x)*sin(currentRad.y);
	M[2][1]=cos(currentRad.x);

	vec2 ba = M*(b-a);
	float phi = atan2(ba.x,ba.y);
	float thetabar = phi - 2*psi/M_PI*atan(e/r);

	//Check For Tacking	
	thetabar = Utility::TackingStrategy(e,phi,windNorth,thetabar,r,psi,ksi,&q);

	//Standard Command for rudder and sail
	vec2 cmdv = Utility::StandardCommand(heading,thetabar, windNorth, M_PI/3.0);
	cmd.angular.x = cmdv.x;
	cmd.angular.y = cmdv.y;

	//LOG
	publishLOG("PLine following Thetabar : " + std::to_string(thetabar) + " Obj : " + std::to_string(waypoints[1].x) + ", " + std::to_string(waypoints[1].y)+ "\ne : " + std::to_string(e));

	return cmd;
}
