#include "identification_node/identification.hpp"
#include <math.h>
#include <fstream>
#include <iostream>
#include <utilities.hpp>
#include <ros/package.h>

using namespace Sailboat;
using namespace glm;

void Identification::setup(ros::NodeHandle* n){
	float currentHeading = (Utility::QuaternionToEuler(imuMsg.orientation)).z;
	initPos = vec2(gpsMsg.latitude, gpsMsg.longitude);

	initWind = Utility::RelativeToTrueWind(vec2(velMsg.linear.x,velMsg.linear.y),currentHeading,windMsg.theta,windMsg.x);

	vec2 current(gpsMsg.latitude, gpsMsg.longitude);
	vec2 initPosXYZ = Utility::GPSToCartesian(current);
	float distGPS = 500;
	float rearth = 6371000;
	float latdist = M_PI/180.0*6378137*cos(initPos.y);
	float longdist = M_PI/180.0*6367449*cos(initPos.x);
	goal1 = vec2(initPos.x + distGPS/latdist*cos(initWind+M_PI/2.0),initPos.y + distGPS/longdist*sin(initWind+M_PI/2.0));
	start = std::clock();
}

geometry_msgs::Twist Identification::control(){
	geometry_msgs::Twist cmd;

	vec2 current(gpsMsg.latitude, gpsMsg.longitude);
	vec2 currentRad = vec2(gpsMsg.latitude*M_PI/180.0, gpsMsg.longitude*M_PI/180.0);
	vec3 currentXYZ = Utility::GPSToCartesian(gpsMsg.latitude, gpsMsg.longitude);

	vec3 heading = Utility::QuaternionToEuler(imuMsg.orientation);
	float currentHeading = heading.z;
	float windNorth = Utility::RelativeToTrueWind(vec2(velMsg.linear.x,velMsg.linear.y),currentHeading,windMsg.theta,windMsg.x);

	vec2 ruddersail;

	float thetabar;
	switch(step){
		case 0:{
			       vec3 b = Utility::GPSToCartesian(goal1);
			       vec3 a = Utility::GPSToCartesian(initPos);
			       float na = glm::length(a);
			       float nb = glm::length(b);

			       float lat1 = initPos.x*M_PI/180.0;
			       float lat2 = goal1.x*M_PI/180.0;
			       float long1 = initPos.y*M_PI/180.0;
			       float long2 = goal1.y*M_PI/180.0;

			       double diflat = lat2-lat1;
			       double diflong = long2 - long1;
			       double leng = (diflat*(currentRad.x-lat1)+diflong*(currentRad.y-long1))/(diflat*diflat+diflong*diflong);
			       vec2 currline(lat1+diflat*leng,long1+diflong*leng);

			       double distToLine = Utility::GPSDist(currline,current);

			       vec3 n = glm::cross(a,b)/(na*nb);

			       float e = glm::dot(currentXYZ,n);
			       e = e>0?distToLine:-distToLine;

			       float phi = Utility::GPSBearing(initPos,goal1);
			       thetabar = phi - 2*M_PI/3.0/M_PI*atan(e/10);

			       //ruddersail = Utility::StandardCommand(currentHeading,thetabar, windNorth, M_PI/6.0);
			       ruddersail = Utility::StandardCommand(heading,initWind+(float)(M_PI/2.0), windNorth, (float)(M_PI/6.0));
			       double duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;

			       if(cos(currentHeading - (initWind+M_PI/2.0)) > 0.9 && duration > 60){
				       step++;
				       start = std::clock();
			       }
			       break;
		       }
		case 1:{
			       ruddersail = Utility::StandardCommand(heading,initWind+(float)(3.0*M_PI/4.0), windNorth, (float)(M_PI/2.0));
			       double duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;

			       if(cos(currentHeading - (initWind+3.0*M_PI/4.0)) > 0.9 && duration > 30){
				       step++;
				       start = std::clock();
			       }
			       break;
		       }
		case 2:{
			       ruddersail.x = M_PI/4.0;
			       ruddersail.y = 0;
			       double duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;

			       if(duration > 10){
				       step++;
				       start = std::clock();
			       }
			       break;
		       }
		case 3:{
			       ruddersail.x = 0;
			       ruddersail.y = 0;
			       double duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;

			       if(duration > 10){
				       step++;
				       start = std::clock();
			       }
			       break;
		       }
	}


	cmd.angular.x = (double)ruddersail.x;
	cmd.angular.y = (double)ruddersail.y;

	std::string message = "";

	publishMSG("step " + std::to_string(step));
	return cmd;
}

