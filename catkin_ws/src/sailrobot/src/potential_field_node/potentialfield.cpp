#include "potential_field_node/potentialfield.hpp"
#include "math.h"
#include <utilities.hpp>

using namespace Sailboat;

void PotentialField::setup(ros::NodeHandle* n){
	subWays = n->subscribe("waypoints", 100, &PotentialField::waypoint_callback, this);
	subObs = n->subscribe("obstacles", 100, &PotentialField::obstacles_callback, this);
}

tf::Vector3 PotentialField::toXYZ(tf::Vector3 pos){
	tf::Vector3 res;	
	return res;
}

tf::Vector3 PotentialField::distanceVector(tf::Vector3 dest, tf::Vector3 pos){
	double d = Utility::GPSDist(pos.getX(), pos.getY(), dest.getX(), dest.getY());
	double bearing = Utility::GPSBearing(pos.getX(), pos.getY(), dest.getX(), dest.getY());

	return tf::Vector3(d*cos(bearing), d*sin(bearing), 0);
}

geometry_msgs::Twist PotentialField::control(){
	if(wpoints.size() < 1)
		return geometry_msgs::Twist();	
	std::cout << "controlling" << std::endl;
	tf::Vector3 current(gpsMsg.latitude, gpsMsg.longitude, 0);

	geometry_msgs::Twist cmd;
	tf::Vector3 heading;

	for(std::vector<geometry_msgs::Point>::iterator it = wpoints.begin(); it != wpoints.end(); ++it){
		heading += distanceVector(current, tf::Vector3((*it).x, (*it).y, (*it).z));
	}
	heading.normalize();

	for(std::vector<geometry_msgs::Point>::iterator it = opoints.begin(); it != opoints.end(); ++it){
		tf::Vector3 res = distanceVector(current,tf::Vector3((*it).x, (*it).y, (*it).z));

		if(float dist = res.length2() < 100){
			res /= dist*dist;
			heading += res;
		}
	}

	cmd.linear.x = (double)heading.getX();
	cmd.linear.y = (double)heading.getY();
	cmd.linear.z = (double)heading.getZ();
	return cmd;
}

void PotentialField::waypoint_callback(const geometry_msgs::Point::ConstPtr& waypoints){
	wpoints.push_back(*waypoints);
}

void PotentialField::obstacles_callback(const geometry_msgs::Point::ConstPtr& obspoints){
	opoints.push_back(*obspoints);
}
