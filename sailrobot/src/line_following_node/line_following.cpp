#include "line_following_node/line_following.hpp"
#include <math.h>
#include <ros/package.h>
#include <utilities.hpp>
#include <glm/glm.hpp>
#include <iostream>
#include <fstream>

using namespace Sailboat;
using namespace glm;

void LineFollowing::setup(ros::NodeHandle* n){
	std::string waypointPath = "data/line_following.txt";
	if(n->hasParam("waypoints"))
		n->getParam("waypoints",waypointPath);
	waypoints.push_back(glm::vec2(gpsMsg.latitude, gpsMsg.longitude));
	waypoints = Utility::AppendGPSCoordinates(waypointPath, nbWaypoints, &waypoints);
	if(nbWaypoints == 0){
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
	//Retrieve data
	vec2 current = vec2(gpsMsg.latitude, gpsMsg.longitude);
	vec2 currentRad = vec2(gpsMsg.latitude*M_PI/180.0, gpsMsg.longitude*M_PI/180.0);
	vec3 currentXYZ = Utility::GPSToCartesian(gpsMsg.latitude, gpsMsg.longitude);
	vec3 heading = Utility::QuaternionToEuler(imuMsg.orientation);
	float windNorth = Utility::RelativeToTrueWind(vec2(velMsg.linear.x,velMsg.linear.y),heading.z,windMsg.theta, windMsg.x, windMsg.y);

	//Parameters
	float psi = M_PI/4.0;
	float ksi = M_PI/3.0;

	/// Calculate the distance to the next waypoint, if close, change to the next waypoint
	float dist = Utility::GPSDist(current, waypoints[(currentWaypoint+1)%nbWaypoints]);
	if(dist < 5){
		publishMSG("PArrived at waypoint " + std::to_string(currentWaypoint));
		currentWaypoint++;
		currentWaypoint %= nbWaypoints;
		if(waypoints.size() > nbWaypoints)
			waypoints.erase(waypoints.begin());
	}

	//Find closest point on line
	float lat1 = waypoints[currentWaypoint].x;
	float lat2 = waypoints[(currentWaypoint+1)%nbWaypoints].x;
	float long1 = waypoints[currentWaypoint].y;
	float long2 = waypoints[(currentWaypoint+1)%nbWaypoints].y;

  	double diflat = lat2-lat1;
  	double diflong = long2 - long1;
  	double leng = (diflat*(current.x-lat1)+diflong*(current.y-long1))/(diflat*diflat+diflong*diflong);
  	vec2 currline(lat1+diflat*leng,long1+diflong*leng);

	double distToLine = Utility::GPSDist(currline,current);

	float phi = Utility::GPSBearing(waypoints[currentWaypoint],waypoints[(currentWaypoint+1)%nbWaypoints]);

	//Which side of the line ?
	float b = Utility::GPSBearing(waypoints[currentWaypoint],current);
	float e = sin(b-phi)>0?distToLine:-distToLine;

	float thetabar = phi - 2*psi/M_PI*atan(e/r);
	//Check For Tacking
	bool check = false;
	thetabar = Utility::TackingStrategy(e,phi,windNorth,thetabar,r,psi,ksi,&q, &check);

	//Standard Command for rudder and sail
	vec2 cmdv = Utility::StandardCommand(heading,thetabar, windNorth, M_PI/3.0);
	cmd.angular.x = cmdv.x;
	cmd.angular.y = cmdv.y;

	//LOG
	std::string checkTacking = "";
	if(check)
		checkTacking = " Tacking! " + std::to_string(q);
	publishLOG("PLine following Thetabar : " + std::to_string(thetabar) + " Obj : " + std::to_string(waypoints[1].x) + ", " + std::to_string(waypoints[1].y)+ "\ne : " + std::to_string(e) +"\nTrue Wind : " + std::to_string(windNorth) + " dist : " + std::to_string(dist) + checkTacking);

	return cmd;
}
