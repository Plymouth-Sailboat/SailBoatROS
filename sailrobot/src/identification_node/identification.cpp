#include "identification_node/identification.hpp"
#include <math.h>
#include <utilities.hpp>
#include <ros/package.h>
#include <nlopt.hpp>
#include <numeric>

using namespace Sailboat;
using namespace glm;

double Identification::costFunction(const std::vector<double> &x, std::vector<double> &grad, void *option){
		std::vector<double*> state = *(std::vector<double*>*)option;
		double p6 = state.back()[0];
		double p7 = state.back()[1];
		double p8 = state.back()[2];
		double p9 = state.back()[3];
		double atw = state.back()[4];
		double dt = 0.1;
		//Reference State gps.x gps.y heading v w dtw aaw daw rud sail
		//TODO

		//Simulate State
		std::vector<double*> predicted;
		double states[5] = {0,0,0,0,0};
		for(int i = 0; i< state.size()-1; ++i){
			double daw = state[i][7];
			double dtw = state[i][5]+M_PI/2.0;
			double aaw = state[i][6];
			double atw = 0;
			double gs = x[3]*aaw*sin(state[i][9]-daw);
			double gr = x[4]*states[3]*states[3]*sin(state[i][8]);

			states[0] += states[3]*cos(states[2])+atw*x[0]*cos(dtw)*dt;
			states[1] += states[3]*sin(states[2])+atw*x[0]*sin(dtw)*dt;
			states[2] += states[4]*dt;
			states[3] += (gs*sin(state[i][9])-gr*x[6]*sin(state[i][8])-x[1]*states[3]*states[3])/p9*dt;
			states[4] += (gs*(p6-p7*cos(state[i][9]))-gr*p8*cos(state[i][8])-x[2]*states[4]*states[3])/x[5]*dt;

			predicted.push_back(states);
		}

		//cost
		double f = 0;
		int it_i = 0;
		for(std::vector<double*>::iterator it=state.begin(); it != state.end()-1; it++){
			double gpsdist = Utility::GPSDist((*it)[0], (*it)[1], 0, 0);
			double xpos = gpsdist*sin((*it)[2]);
			double ypos = -gpsdist*cos((*it)[2]);
			f += 1*(xpos - predicted[it_i][0])*(xpos - predicted[it_i][0]);
			f += 1*(ypos - predicted[it_i][1])*(ypos - predicted[it_i][1]);
			f += 1*sin((*it)[2]-M_PI/2.0 - predicted[it_i][2])*sin((*it)[2]-M_PI/2.0 - predicted[it_i][2]);
			f += 1*((*it)[3] - predicted[it_i][3])*((*it)[3] - predicted[it_i][3]);
			f += 0.2*((*it)[4] - predicted[it_i][4])*((*it)[4] - predicted[it_i][4]);
			it_i++;
		}

		return f;
}

void Identification::setup(ros::NodeHandle* n){
	float currentHeading = (Utility::QuaternionToEuler(imuMsg.orientation)).z;
	initPos = vec2(gpsMsg.latitude, gpsMsg.longitude);

	initWind = Utility::RelativeToTrueWind(vec2(velMsg.linear.x,velMsg.linear.y),currentHeading,windMsg.theta, windMsg.x, windMsg.y);
	initWindA = sqrt(windMsg.x*windMsg.x+windMsg.y*windMsg.y);
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
	data << "time,clock,step,dvx,dvy,dvz,ax,ay,az,avx,avy,avz,heading,twind,windacc,awind,cmdrudder,cmdsail,lat,long" << std::endl;
}

geometry_msgs::Twist Identification::control(){
	geometry_msgs::Twist cmd;

	vec2 current(gpsMsg.latitude, gpsMsg.longitude);

	vec3 heading = Utility::QuaternionToEuler(imuMsg.orientation);
	float currentHeading = heading.z;
	float windNorth = Utility::RelativeToTrueWind(vec2(velMsg.linear.x,velMsg.linear.y),currentHeading,windMsg.theta, windMsg.x, windMsg.y);

	vec2 ruddersail;

	float thetabar;

	std::vector<double*> state;
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
		std::to_string(sqrt(windMsg.x*windMsg.x+windMsg.y*windMsg.y)) << "," <<
		std::to_string(windMsg.theta) << "," <<
		std::to_string(ruddersail.x) << "," <<
		std::to_string(ruddersail.y) << "," <<
		std::to_string(current.x) << "," <<
		std::to_string(current.y) << std::endl;

		double vnorm = sqrt(velMsg.linear.x*velMsg.linear.x+velMsg.linear.y*velMsg.linear.y);
		state.push_back(new double[10]{current.x-initPos.x,current.y-initPos.y,currentHeading,vnorm,imuMsg.angular_velocity.z,windNorth,sqrt(windMsg.x*windMsg.x+windMsg.y*windMsg.y),windMsg.theta,ruddersail.x,ruddersail.y});

		cmd.angular.x = (double)ruddersail.x;
		cmd.angular.y = (double)ruddersail.y;

		std::string message = "";

		publishLOG("step " + std::to_string(step));
	}


	if(step == 4){
		if(data.is_open())
			data.close();

		std::string path = ros::package::getPath("sailrobot");
		std::ifstream infile(path+"/data/identification.txt");
		std::string line;

		//Approximate parameters values
		//std::vector<std::vector<double>> datavec;
		std::vector<double> datain;
		std::vector<double> s1;
		std::vector<double> s2;
		std::vector<double> s3;
		std::vector<double> s4;
		std::vector<double> s5;
		std::vector<double> p1(1,0.0);
		double maxv = 0;
		double prevheading = 0;
		while(std::getline(infile,line)){
			std::stringstream ss(line);

			for (double i; ss >> i;) {
			  datain.push_back(i);
			  if (ss.peek() == ',')
			      ss.ignore();
			}

			double aaw = datain[14];
			double daw = datain[15];
			double angacc = 0;
			double angvel = datain[12];
			if(prevheading != 0)
				angacc = (datain[12]-prevheading)*0.1;
			prevheading = datain[12];

			double vnorm = sqrt(datain[3]*datain[3]+datain[4]*datain[4]);
			double anorm = sqrt(datain[6]*datain[6]+datain[7]*datain[7]);

			double p10 = stod(Utility::Instance().config["p10"]);
			double p3 = stod(Utility::Instance().config["p3"]);
			double p8 = stod(Utility::Instance().config["p8"]);
			double p9 = stod(Utility::Instance().config["p9"]);

			switch((int)datain[2]){
				case 0://S2 p1
					if(vnorm > maxv)
						maxv = vnorm;
					s2.push_back(anorm/(vnorm*vnorm));
				break;
				case 2://S3 S1
					s1.push_back(maxv*maxv/(aaw*sin(datain[17]-daw)*sin(datain[17])));
					s3.push_back(2*(p10*angacc+p3*angvel*vnorm)/(vnorm*vnorm*p8));
				break;
				case 3://S4 S5
					s4.push_back(-angacc/(angvel*vnorm));
					s5.push_back(-anorm/(vnorm*vnorm)*p9);
				break;
			}
			datain.clear();
		}
		infile.close();

		float s1p = std::accumulate(std::begin(s1), std::end(s1), 0.0) / s1.size();
		float s2p = std::accumulate(std::begin(s2), std::end(s2), 0.0) / s2.size();
		float s3p = std::accumulate(std::begin(s3), std::end(s3), 0.0) / s3.size();
		float s4p = std::accumulate(std::begin(s4), std::end(s4), 0.0) / s4.size();
		float s5p = std::accumulate(std::begin(s5), std::end(s5), 0.0) / s5.size();
		float p1p = std::accumulate(std::begin(p1), std::end(p1), 0.0) / p1.size();

		double p6 = stod(Utility::Instance().config["p6"]);
		double p7 = stod(Utility::Instance().config["p7"]);
		double p8 = stod(Utility::Instance().config["p8"]);
		double p9 = stod(Utility::Instance().config["p9"]);
		double p10 = stod(Utility::Instance().config["p10"]);

		s1.clear();
		s2.clear();
		s3.clear();
		s4.clear();
		s5.clear();
		p1.clear();
		state.push_back(new double[10]{p6,p7,p8,p9,0,0,0,0,0,0});

		//Regression step for parameters
		nlopt::opt opt(nlopt::GN_DIRECT_L, 7);
		opt.set_min_objective(costFunction, (void*)&state);

		std::vector<double> lb(7,0.0);
		opt.set_lower_bounds(lb);
		std::vector<double> ub(7,10000.0);
		opt.set_upper_bounds(ub);
		std::vector<double> x{p1p,s5p,s4p*p10,s1p*s5p,s3p,p10,(p9*s2p+2*s5p)/s3p};

		double minf;
		for(int i = 0; i < 10; i++){
			try{
		    		nlopt::result result = opt.optimize(x, minf);
		    		std::cout << "found minimum at f(";
						for(int i = 0; i < 7; ++i)
						 	std::cout << x[i] << ",";
						std::cout << ") = " << std::setprecision(10) << minf << std::endl;
			}
			catch(std::exception &e) {
		    		std::cout << "nlopt failed: " << e.what() << std::endl;
			}
		}
	}
	return cmd;
}
