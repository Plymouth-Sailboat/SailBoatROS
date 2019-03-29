#include "user_friendly_test_node/user_friendly_test.hpp"
#include "math.h"
#include <fstream>
#include <iostream>
#include <utilities.hpp>
#include <ros/package.h>

using namespace Sailboat;
using namespace glm;

void UserTest::setup(ros::NodeHandle* n){
}

bool UserTest::loopUnpublished(){
	float lat = gpsMsg.latitude;
	float lon = gpsMsg.longitude;
	float alt = gpsMsg.altitude;
	float speed = gpsMsg.speed;
	float time = gpsMsg.time;
	vec3 euler = Utility::QuaternionToEuler(imuMsg.orientation);
	vec2 wind = vec2(windMsg.x, windMsg.y);
	float windSpeed = length(wind);

	std::cout << "data : " << std::endl << std::endl;
	std::cout << "gps (lat,long,alt) : (" << lat << ", " << lon << ", " << alt << ")" << std::endl;
	std::cout << "gps speed : " << speed << std::endl;
	std::cout << "gps time  : " << time << std::endl << std::endl;
	std::cout << "boat heading : " << euler.z*180.0/M_PI << std::endl << std::endl;
	std::cout << "wind direction : " << windMsg.theta*180.0/M_PI << std::endl;
	std::cout << "wind anemometer : " << windSpeed << "m/s" << std::endl << std::endl;
	std::cout << "rudder angle : " << rudderAngle << std::endl;
	std::cout << "sail angle : " << sailAngle << std::endl;

	return true;
}

