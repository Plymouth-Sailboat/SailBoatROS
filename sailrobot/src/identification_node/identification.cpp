#include "identification_node/identification.hpp"
#include <math.h>
#include <utilities.hpp>
#include <ros/package.h>
#include <nlopt.hpp>

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
	float latdist = 11132.92-559.82*cos(2*initPos.x*180.0/M_PI)+1.175*cos(4*initPos.x*180.0/M_PI)-0.0023*cos(6*initPos.x*180.0/M_PI);
	float longdist = M_PI/180.0*6367449*cos(initPos.x*180.0/M_PI);
	goal1 = vec2(initPos.x + distGPS/latdist*cos(initWind+M_PI/2.0),initPos.y + distGPS/longdist*sin(initWind+M_PI/2.0));
	start = ros::Time::now().toSec();

	std::string path = ros::package::getPath("sailrobot");
	data.open(path+"/data/identification.txt");
	data << "time,clock,step,dvx,dvy,dvz,ax,ay,az,heading,wind,cmdrudder,cmdsail,lat,long" << std::endl;
}

geometry_msgs::Twist Identification::control(){
	geometry_msgs::Twist cmd;

	vec2 current(gpsMsg.latitude, gpsMsg.longitude);

	vec3 heading = Utility::QuaternionToEuler(imuMsg.orientation);
	float currentHeading = heading.z;
	float windNorth = Utility::RelativeToTrueWind(vec2(velMsg.linear.x,velMsg.linear.y),currentHeading,windMsg.theta,windMsg.x);

	vec2 ruddersail;

	float thetabar;
	switch(step){
		case 0:{
			float lat1 = initPos.x;
			float lat2 = goal1.x;
			float long1 = initPos.y;
			float long2 = goal1.y;

			double diflat = lat2-lat1;
			double diflong = long2 - long1;
			double leng = (diflat*(current.x-lat1)+diflong*(current.y-long1))/(diflat*diflat+diflong*diflong);
			vec2 currline(lat1+diflat*leng,long1+diflong*leng);

			double distToLine = Utility::GPSDist(currline,current);

			float b = Utility::GPSBearing(initPos,current);

			float phi = Utility::GPSBearing(initPos,goal1);
			float e = sin(b-phi)>0?distToLine:-distToLine;
			thetabar = phi - 2*M_PI/3.0/M_PI*atan(e/10);

			ruddersail = Utility::StandardCommand(heading,thetabar, windNorth, M_PI/6.0);
			//ruddersail = Utility::StandardCommand(heading,initWind+(float)(M_PI/2.0), windNorth, (float)(M_PI/6.0));
			double duration  = ros::Time::now().toSec() - start;

			if(cos(currentHeading - (initWind+M_PI/2.0)) > 0.7 && duration > 60){
				step++;
				start = ros::Time::now().toSec();
			}
			break;
		}
		case 1:{
			ruddersail = Utility::StandardCommand(heading,initWind+(float)(3.0*M_PI/4.0), windNorth, (float)(M_PI/2.0));
			double duration  = ros::Time::now().toSec() - start;

			if(cos(currentHeading - (initWind+3.0*M_PI/4.0)) > 0.9 && duration > 10){
				step++;
				start = ros::Time::now().toSec();
			}
			break;
		}
		case 2:{
			       ruddersail.x = M_PI/4.0;
			       ruddersail.y = 0;
			       double duration  = ros::Time::now().toSec() - start;

			       if(duration > 5){
				       step++;
				       start = ros::Time::now().toSec();
			       }
			       break;
		       }
		case 3:{
			       ruddersail.x = 0;
			       ruddersail.y = 0;
			       double duration  = ros::Time::now().toSec() - start;

			       if(duration > 5){
				       step++;
				       start = ros::Time::now().toSec();
			       }
			       break;
		       }
	}

	if(step < 4)
		data << std::to_string(ros::Time::now().toSec()) << "," << std::to_string(ros::Time::now().toSec() - start) << "," << std::to_string(step) << "," << std::to_string(velMsg.linear.x) << "," << std::to_string(velMsg.linear.y) << "," << std::to_string(velMsg.linear.z) << "," << std::to_string(imuMsg.linear_acceleration.x) << "," << std::to_string(imuMsg.linear_acceleration.y) << "," << std::to_string(imuMsg.linear_acceleration.z) << "," << std::to_string(currentHeading) << "," << std::to_string(windNorth) << "," << std::to_string(ruddersail.x) << "," << std::to_string(ruddersail.y) << "," << std::to_string(current.x) << "," << std::to_string(current.y) << std::endl; 

	cmd.angular.x = (double)ruddersail.x;
	cmd.angular.y = (double)ruddersail.y;

	std::string message = "";

	publishLOG("step " + std::to_string(step));
	return cmd;
}

