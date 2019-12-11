#include "identification_node/identification.hpp"
#include <math.h>
#include <utilities.hpp>
#include <ros/package.h>
#include <nlopt.hpp>
#include <numeric>
#include <array>

using namespace Sailboat;
using namespace glm;

double Identification::costFunction(const std::vector<double> &x, std::vector<double> &grad, void *option){
	std::vector<std::array<double,10>> state = *(std::vector<std::array<double,10>>*)option;
	double p6 = state.back()[0];
	double p7 = state.back()[1];
	double p8 = state.back()[2];
	double p9 = state.back()[3];
	double atw = state.back()[4];
	double initV = state.back()[5];
	double initTheta = state.back()[6];
	double dt = 0.01;
	//Reference State gps.x gps.y heading v w dtw aaw daw rud sail
	//TODO

	//Simulate State
	std::vector<std::array<double,5>> predicted;
	std::array<double,5> states{0,0,initTheta,initV,0};
	for(int i = 0; i< state.size()-1; ++i){
		double daw = state[i][7];
		double dtw = state[i][5];
		double aaw = state[i][6];
		double sigma = cos(dtw)+cos(state[i][9]);
		double delta_s = 0;
		if (sigma<0){
			delta_s = M_PI + dtw;
		}
		else{
			delta_s = -sign(sin(dtw))*state[i][9];
		}
		double gs = x[3]*aaw*sin(delta_s-daw);
		double gr = x[4]*states[3]*states[3]*sin(state[i][8]);

		for(int j = 0; j < 10; ++j){
			states[0] += (states[3]*cos(states[2])+atw*x[0]*cos(dtw))*dt;
			states[1] += (states[3]*sin(states[2])+atw*x[0]*sin(dtw))*dt;
			states[2] += states[4]*dt;
			states[3] += (gs*sin(delta_s)-gr*x[6]*sin(state[i][8])-x[1]*states[3]*states[3])/p9*dt;
			states[4] += (gs*(p6-p7*cos(delta_s))-gr*p8*cos(state[i][8])-x[2]*states[4]*states[3])/x[5]*dt;
			states[2]=mod(states[2],2*M_PI);
		}
		predicted.push_back(states);
	}

	//cost
	double f = 0;
	int it_i = 0;
	for(std::vector<std::array<double,10>>::iterator it=state.begin(); it != state.end()-1; it++){
		double gpsdist = Utility::GPSDist((*it)[0], (*it)[1], 0, 0);
		double bearing = Utility::GPSBearing(0,0,(*it)[0],(*it)[1]);
		double ypos = gpsdist*cos(bearing);
		double xpos = gpsdist*sin(bearing);
		f += 1*(xpos - predicted[it_i][0])*(xpos - predicted[it_i][0]);
		f += 1*(ypos - predicted[it_i][1])*(ypos - predicted[it_i][1]);
		f += 10*(sin((*it)[2] - predicted[it_i][2]))*sin(((*it)[2] - predicted[it_i][2]));
		f += 1*((*it)[3] - predicted[it_i][3])*((*it)[3] - predicted[it_i][3]);
		//f += 0.2*((*it)[4] - predicted[it_i][4])*((*it)[4] - predicted[it_i][4]);

		//std::cout << (*it)[2]<< "," << predicted[it_i][2] << " " <<ypos << "," <<  predicted[it_i][1] << std::endl;
		it_i++;
	}
	//exit(0);
	return f;
}

void Identification::setup(ros::NodeHandle* n){
	float currentHeading = (Utility::QuaternionToEuler(imuMsg.orientation)).z;
	initPos = dvec2(gpsMsg.latitude, gpsMsg.longitude);
	initXRef = gpsMsg.latitude;
	initYRef = gpsMsg.longitude;

	initWind = Utility::RelativeToTrueWind(dvec2(velMsg.linear.x,velMsg.linear.y),currentHeading,windMsg.theta, windMsg.x, windMsg.y);
	initWindA = sqrt(windMsg.x*windMsg.x+windMsg.y*windMsg.y);
	double vnorm = sqrt(velMsg.linear.x*velMsg.linear.x+velMsg.linear.y*velMsg.linear.y);
	initV = vnorm;
	initTheta = currentHeading;

	float distGPS = 500;
	float rearth = 6371000;
	float latdist = 11132.92-559.82*cos(2*initPos.x*180.0/M_PI)+1.175*cos(4*initPos.x*180.0/M_PI)-0.0023*cos(6*initPos.x*180.0/M_PI);
	float longdist = M_PI/180.0*6367449*cos(initPos.x*180.0/M_PI);
	goal1 = dvec2(initPos.x + distGPS/latdist*cos(initWind+M_PI/2.0),initPos.y + distGPS/longdist*sin(initWind+M_PI/2.0));
	start = ros::Time::now().toSec();

	std::string path = ros::package::getPath("sailrobot");
	//n->param("simu", doSimu, 0);
	//if(!doSimu){
	//	step = 4;
	//	std::cout << "Not doing Scenario" << std::endl;
	//}else{
	data.open(path+"/data/identification.csv");
	data << "time,clock,step,dvx,dvy,dvz,ax,ay,az,avx,avy,avz,heading,twind,twinda,windacc,awind,cmdrudder,cmdsail,lat,long,vgps,thetagps" << std::endl;
	dataState.open(path+"/data/identification_state.csv");
	dataState << "lat,long,heading,v,vgps,thetagps,dtheta,windN,windA,windT,rudder,sail" << std::endl;
	//}
}

geometry_msgs::Twist Identification::control(){

	geometry_msgs::Twist cmd;

	dvec2 current(gpsMsg.latitude, gpsMsg.longitude);

	vec3 heading = Utility::QuaternionToEuler(imuMsg.orientation);
	float currentHeading = heading.z;
	float windNorthA = 0.0;
	float windNorth = Utility::RelativeToTrueWind(dvec2(velMsg.linear.x,velMsg.linear.y),currentHeading,windMsg.theta, windMsg.x, windMsg.y, &windNorthA);
	double vnorm = sqrt(velMsg.linear.x*velMsg.linear.x+velMsg.linear.y*velMsg.linear.y);
	dvec2 ruddersail;
	double duration  = ros::Time::now().toSec() - start;

	float thetabar;


	switch(step){
		case 0:{
			vnorm_list.push_back(vnorm);
			ruddersail = Utility::StandardCommand(heading,initWind+(float)(M_PI/4.0), windNorth, (float)(M_PI/2.0));
			bool vnormStab = false;
			if(vnorm_list.size() > 10){
				float vnormacc = 0.0;
				for(int i = vnorm_list.size()-1; i > vnorm_list.size()-11; i--){
					vnormacc += vnorm_list[i] - vnorm_list[i-1];
				}
				vnormStab = abs(vnormacc/10.0) < 0.0001;
			}
			if(cos(currentHeading - (initWind+M_PI/4.0)) > 0.7 && (duration > 30 || (duration > 10 && vnormStab))){
				step++;
				start = ros::Time::now().toSec();
			}
			break;
		}
		case 1:{
			ruddersail = Utility::StandardCommand(heading,initWind+(float)(M_PI/2.0)+0.01, windNorth, (float)(M_PI/2.0));
			//ruddersail = Utility::StandardCommand(heading,initWind+(float)(M_PI/4.0)+0.01, windNorth, (float)(M_PI/2.0));

			if(cos(currentHeading - (initWind+M_PI/2.0)) > 0.7 && duration > 10){
				step++;
				start = ros::Time::now().toSec();
			}
			break;
		}
		case 2:{
			ruddersail.x = M_PI/4.0;
			ruddersail.y = 0.0;

			if(duration > 5){
				step++;
				start = ros::Time::now().toSec();
			}
			break;
		}
		case 3:{
			ruddersail.x = 0;
			ruddersail.y = 0;

			if(duration > 5){
				step++;
				start = ros::Time::now().toSec();
			}
			break;
		}
	}

	if(step < 4){
		data << std::to_string(ros::Time::now().toSec()) << "," <<
		std::to_string(ros::Time::now().toSec() - start) << "," <<
		std::to_string(step) << "," <<
		std::to_string(velMsg.linear.x) << "," <<
		std::to_string(velMsg.linear.y) << "," <<
		std::to_string(velMsg.linear.z) << "," <<
		std::to_string(imuMsg.linear_acceleration.x) << "," <<
		std::to_string(imuMsg.linear_acceleration.y) << "," <<
		std::to_string(imuMsg.linear_acceleration.z) << "," <<
		std::to_string(imuMsg.angular_velocity.x) << "," <<
		std::to_string(imuMsg.angular_velocity.y) << "," <<
		std::to_string(imuMsg.angular_velocity.z) << "," <<
		std::to_string(currentHeading) << "," <<
		std::to_string(windNorth) << "," <<
		std::to_string(windNorthA) << "," <<
		std::to_string(sqrt(windMsg.x*windMsg.x+windMsg.y*windMsg.y)) << "," <<
		std::to_string(windMsg.theta) << "," <<
		std::to_string(ruddersail.x) << "," <<
		std::to_string(ruddersail.y) << "," <<
		std::to_string(gpsMsg.latitude) << "," <<
		std::to_string(gpsMsg.longitude) << "," <<
		std::to_string(gpsMsg.speed) << "," <<
		std::to_string(gpsMsg.track) << std::endl;



		double vnorm = sqrt(velMsg.linear.x*velMsg.linear.x+velMsg.linear.y*velMsg.linear.y);
		std::array<double,12> dataPushed{gpsMsg.latitude,gpsMsg.longitude,currentHeading,vnorm,gpsMsg.speed, gpsMsg.track,imuMsg.angular_velocity.z,windNorth,sqrt(windMsg.x*windMsg.x+windMsg.y*windMsg.y),windMsg.theta,ruddersail.x,ruddersail.y};
		state.push_back(dataPushed);

		for(int i = 0; i < 12; ++i)
		dataState << std::to_string(dataPushed[i]) << ",";
		dataState << std::endl;

		cmd.angular.x = (double)ruddersail.x;
		cmd.angular.y = (double)ruddersail.y;

		std::string message = "";

		publishLOG("step " + std::to_string(step) + " duration " + std::to_string(duration) + " vnorm " + std::to_string(vnorm) + " head-wind " + std::to_string(cos(currentHeading - (initWind+M_PI/2.0))));
	}
	if(step==4){
		dataState << "0,0,0,0,0," << initV << ",0,0," << initTheta << ",0,0,0" << std::endl;
		if(data.is_open())
			data.close();
		if(dataState.is_open())
			dataState.close();
		exit(0);
	}
	return cmd;
}
