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
	std::string path = ros::package::getPath("sailrobot");
	std::ifstream f(path + "/data/line_following.txt");
	if(f.good()){
		f.close();
		waypoints = Utility::ReadGPSCoordinates(path + "/data/waypoints.txt");
	}else{
		std::cerr << "Waypoints Coordinates File not Found" << std::endl;
		exit(0);
	}
	
	int rowsWaypoints =  sizeof(waypoints) / sizeof(waypoints[0]);
	if(rowsWaypoints > 2){
		std::cerr << "Too Many Coordinates" << std::endl;
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
	
	return cmd;
}
