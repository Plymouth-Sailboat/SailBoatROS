#include "potential_field_node/potentialfield.hpp"
#include "math.h"

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
	int earth_r = 6371000;

	tf::Vector3 res;
	double longDif = (dest.getY() - pos.getY())*M_PI/180.0;
	double latDif = (dest.getX() - pos.getX())*M_PI/180.0;

	double lat1 = pos.getX()*M_PI/180.0;
	double lat2 = dest.getX()*M_PI/180.0;
	double long1 = pos.getY()*M_PI/180.0;
	double long2 = dest.getY()*M_PI/180.0;

	
	double a = sin(latDif/2.0)*sin(latDif/2.0) +
		cos(lat1) * cos(lat2) *
		sin(longDif/2.0) * sin(longDif/2.0);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	double d = earth_r*c;


	double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(long2-long1);
	double y = sin(long2-long1) * cos(lat2);
	double bearing = atan2(y,x);

	res.setX(d*sin(bearing));
	res.setY(d*cos(bearing));

	return res;
}

geometry_msgs::Twist PotentialField::control(){
	if(wpoints.size() < 1)
		return geometry_msgs::Twist();	
	tf::Vector3 current(gpsMsg.latitude, gpsMsg.longitude, 0);

	geometry_msgs::Twist cmd;
	tf::Vector3 heading;

	for(std::vector<geometry_msgs::Point>::iterator it = wpoints.begin(); it != wpoints.end(); ++it){
		tf::Vector3 goal((*it).x, (*it).y, (*it).z);
		heading += distanceVector(goal,current);
	}
	heading.normalize();

	for(std::vector<geometry_msgs::Point>::iterator it = opoints.begin(); it != opoints.end(); ++it){
		tf::Vector3 obs((*it).x, (*it).y, (*it).z);

		tf::Vector3 res = distanceVector(current,obs);

		if(float dist = res.length2() < 100){
			res /= dist*dist;
			heading += res;
		}
	}

	cmd.linear.x = (double)heading.getX();
	cmd.linear.y = (double)heading.getY();
	cmd.linear.z = (double)heading.getZ();
}

void PotentialField::waypoint_callback(const geometry_msgs::Point::ConstPtr& waypoints){
	wpoints.push_back(*waypoints);
}

void PotentialField::obstacles_callback(const geometry_msgs::Point::ConstPtr& obspoints){
	opoints.push_back(*obspoints);
}
