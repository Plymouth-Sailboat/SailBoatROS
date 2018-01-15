#include "potential_field_node/potentialfield.hpp"

using namespace Sailboat;

void PotentialField::setup(ros::NodeHandle* n){
	subWays = n->subscribe("waypoints", 100, &PotentialField::waypoint_callback, this);
	subObs = n->subscribe("obstacles", 100, &PotentialField::obstacles_callback, this);
}

tf::Vector3 PotentialField::toXY(tf::Vector3 pos){
	tf::Vector3 res;
	return res;
}

geometry_msgs::Twist PotentialField::control(){
	tf::Vector3 current(gpsMsg->latitude, gpsMsg->longitude, 0);
	current = toXY(current);

	geometry_msgs::Twist cmd;
	tf::Vector3 heading;

	for(std::vector<geometry_msgs::Point>::iterator it = wpoints.begin(); it != wpoints.end(); ++it){
		tf::Vector3 goal((*it).x, (*it).y, (*it).z);
		goal = toXY(goal);
		heading += goal - current;
	}
	heading.normalize();

	for(std::vector<geometry_msgs::Point>::iterator it = opoints.begin(); it != opoints.end(); ++it){
		tf::Vector3 obs((*it).x, (*it).y, (*it).z);
		obs = toXY(obs);

		if(float dist = current.distance2(obs) < 100){
                	tf::Vector3 res = current-obs;
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
