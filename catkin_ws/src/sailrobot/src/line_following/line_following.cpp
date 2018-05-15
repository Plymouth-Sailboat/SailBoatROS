#include "line_following_node/line_following.hpp"
#include "math.h"
#include <utilities.hpp>
#include <glm/vec2.hpp>

using namespace Sailboat;
using namespace glm;

void LineFollowing::setup(ros::NodeHandle* n){
	std::string path = ros::package::getPath("sailrobot");
	ifstream f(path + "/data/line_following.txt");
    if(f.good())
		waypoints = Utility::ReadGPSCoordinates(path + "/data/waypoints.txt");
	else{
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
	geometry_msgs::Vector3 heading = Utility::QuaternionToEuler(imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w);
	
	int q = 0;
	float r = 50.0;
	float psi = M_PI/4.0;
	vec2 waypoint1 = vec2(waypoints[0][0], waypoints[0][1]);
	vec2 waypoint2 = vec2(waypoints[1][0], waypoints[1][1]);
	vec2 ba = waypoint2-waypoint1;
	float norm = norm(ba);
	
	float e = determinant((waypoint2-waypoint1)/norm, current-waypoint1);
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
