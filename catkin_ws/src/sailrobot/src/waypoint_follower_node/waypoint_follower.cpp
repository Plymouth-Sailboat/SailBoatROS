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
	std::string path = ros::package::getPath("sailrobot");
	std::ifstream f(path + "/data/waypoints.txt");
	if(f.good()){
		f.close();
		waypoints = Utility::ReadGPSCoordinates(path + "/data/waypoints.txt", nbWaypoints);
	}else{
		std::cerr << "Waypoints Coordinates File not Found" << std::endl;
		exit(0);
	}
	currentWaypoint = 0;
}

geometry_msgs::Twist WaypointFollower::control(){
	geometry_msgs::Twist cmd;

	vec2 current = vec2(gpsMsg.latitude, gpsMsg.longitude);

	if(Utility::GPSDist(current, waypoints[currentWaypoint]) < 10){
		currentWaypoint++;
		publishMSG("PArrive au point " + std::to_string(currentWaypoint));
	}
	if(currentWaypoint > nbWaypoints)
		currentWaypoint = 0;

	float wind = windMsg.theta;
	vec3 heading = Utility::QuaternionToEuler(imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w);
	float theta = Utility::GPSBearing(current, waypoints[currentWaypoint]);

	cmd.angular.z = theta;

	return cmd;
}
