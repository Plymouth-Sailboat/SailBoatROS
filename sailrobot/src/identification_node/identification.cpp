#include "identification_node/identification.hpp"
#include <math.h>
#include <fstream>
#include <iostream>
#include <utilities.hpp>
#include <ros/package.h>

using namespace Sailboat;
using namespace glm;

void Identification::setup(ros::NodeHandle* n){
	initPos = vec2(gpsMsg.latitude, gpsMsg.longitude);
	initWind = windMsg.theta;

	vec2 initPosXYZ = Utility::GPSToCartesian(current);
	goal1 = 
}

geometry_msgs::Twist Identification::control(){
	geometry_msgs::Twist cmd;
	
	vec2 current(gpsMsg.latitude, gpsMsg.longitude);
	float currentHeading = (Utility::QuaternionToEuler(imuMsg.orientation)).z;
	vec2 ruddersail;

	switch(step){
		case 0:

		break;
		case 1:
		break;
		case 2:
		break;
		case 3:
		break;
		case 4:
		break;
		case 5:
		break;
		case 6:
		break;
		case 7:
		break;
		case 8:
		break;
	}


	cmd.angular.x = (double)ruddersail.x;
	cmd.angular.y = (double)ruddersail.y;

	std::string message = "";

	publishMSG("step " + std::to_string(step));
	return cmd;
}

