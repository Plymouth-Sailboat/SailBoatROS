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
	initPos = vec2(gpsMsg.latitude, gpsMsg.longitude);
	initXRef = gpsMsg.latitude;
	initYRef = gpsMsg.longitude;

	initWind = Utility::RelativeToTrueWind(vec2(velMsg.linear.x,velMsg.linear.y),currentHeading,windMsg.theta, windMsg.x, windMsg.y);
	initWindA = sqrt(windMsg.x*windMsg.x+windMsg.y*windMsg.y);
	double vnorm = sqrt(velMsg.linear.x*velMsg.linear.x+velMsg.linear.y*velMsg.linear.y);
	initV = vnorm;
	initTheta = currentHeading;

	float distGPS = 500;
	float rearth = 6371000;
	float latdist = 11132.92-559.82*cos(2*initPos.x*180.0/M_PI)+1.175*cos(4*initPos.x*180.0/M_PI)-0.0023*cos(6*initPos.x*180.0/M_PI);
	float longdist = M_PI/180.0*6367449*cos(initPos.x*180.0/M_PI);
	goal1 = vec2(initPos.x + distGPS/latdist*cos(initWind+M_PI/2.0),initPos.y + distGPS/longdist*sin(initWind+M_PI/2.0));
	start = ros::Time::now().toSec();

	std::string path = ros::package::getPath("sailrobot");
	//n->param("simu", doSimu, 0);
	//if(!doSimu){
	//	step = 4;
	//	std::cout << "Not doing Scenario" << std::endl;
	//}else{
	data.open(path+"/data/identification.csv");
	data << "time,clock,step,dvx,dvy,dvz,ax,ay,az,avx,avy,avz,heading,twind,twinda,windacc,awind,cmdrudder,cmdsail,lat,long" << std::endl;
	dataState.open(path+"/data/identification_state.csv");
	dataState << "lat,long,heading,v,dtheta,windN,windA,windT,rudder,sail" << std::endl;
	//}
}

geometry_msgs::Twist Identification::control(){

	geometry_msgs::Twist cmd;

	vec2 current(gpsMsg.latitude, gpsMsg.longitude);

	vec3 heading = Utility::QuaternionToEuler(imuMsg.orientation);
	float currentHeading = heading.z;
	float windNorthA = 0.0;
	float windNorth = Utility::RelativeToTrueWind(vec2(velMsg.linear.x,velMsg.linear.y),currentHeading,windMsg.theta, windMsg.x, windMsg.y, &windNorthA);

	vec2 ruddersail;
	double duration  = ros::Time::now().toSec() - start;

	float thetabar;

	switch(step){
		case 0:{ruddersail = Utility::StandardCommand(heading,initWind+(float)(M_PI/4.0), windNorth, (float)(M_PI/2.0));

			if(cos(currentHeading - (initWind+M_PI/4.0)) > 0.7 && duration > 30){
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
		std::to_string(gpsMsg.longitude) << std::endl;



		double vnorm = sqrt(velMsg.linear.x*velMsg.linear.x+velMsg.linear.y*velMsg.linear.y);
		std::array<double,10> dataPushed{gpsMsg.latitude,gpsMsg.longitude,currentHeading,vnorm,imuMsg.angular_velocity.z,windNorth,sqrt(windMsg.x*windMsg.x+windMsg.y*windMsg.y),windMsg.theta,ruddersail.x,ruddersail.y};
		state.push_back(dataPushed);

		for(int i = 0; i < 10; ++i)
		dataState << std::to_string(dataPushed[i]) << ",";
		dataState << std::endl;

		cmd.angular.x = (double)ruddersail.x;
		cmd.angular.y = (double)ruddersail.y;

		std::string message = "";

		publishLOG("step " + std::to_string(step) + " duration " + std::to_string(duration) + " vnorm " + std::to_string(vnorm) + " head-wind " + std::to_string(cos(currentHeading - (initWind+M_PI/2.0))));
	}


	if(step == 4){
		dataState << "0,0,0,0,0," << initV << "," << initTheta << ",0,0,0" << std::endl;
		if(data.is_open())
		data.close();
		if(dataState.is_open())
		dataState.close();
		exit(0);

		std::string path = ros::package::getPath("sailrobot");
		std::ifstream infile(path+"/data/identification.csv");
		std::string line;

		//Approximate parameters values
		//std::vector<std::vector<double>> datavec;
		std::vector<double> datain;
		std::vector<double> s1;
		std::vector<double> s2;
		std::vector<double> s3;
		std::vector<double> s4;
		std::vector<double> s5;
		std::vector<double> p1(1,0.3);
		std::vector<double> atwv;
		double maxv = 0;
		double prevheading = 0;
		std::getline(infile,line);
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
			if(vnorm == 0)
			vnorm = 0.01;
			//	double anorm = sqrt(datain[6]*datain[6]+datain[7]*datain[7]);
			double anorm = datain[7];
			double p10 = stod(Utility::Instance().config["p10"]);
			double p3 = stod(Utility::Instance().config["p3"]);
			double p8 = stod(Utility::Instance().config["p8"]);
			double p9 = stod(Utility::Instance().config["p9"]);

			switch((int)datain[2]){
				case 0://S2 p1
				if(vnorm > maxv)
				maxv = vnorm;
				s1.push_back((aaw*sin(datain[17]-daw)*sin(datain[17])));
				break;
				case 1:
				s2.push_back(anorm/(vnorm*vnorm));
				s3.push_back(2*(p10*angacc+p3*angvel*vnorm)/(vnorm*vnorm*p8));
				break;
				case 2://S3 S1
				s4.push_back(-angacc/(angvel*vnorm));
				break;
				case 3://S4 S5
				s5.push_back(-anorm/(vnorm*vnorm)*p9);
				break;
			}
			atwv.push_back(aaw);
			datain.clear();
		}
		infile.close();

		std::cout << "Starting Optimization" << std::endl;
		publishLOG("Starting Optimization");
		for(int i = 0; i < s1.size(); ++i)
		s1[i] = maxv*maxv/s1[i];
		float s1p = std::accumulate(std::begin(s1)+50, std::end(s1), 0.0) / (s1.size()-50.0);
		s1p = s1p>0?s1p:0.1;
		float s2p = std::accumulate(std::begin(s2), std::end(s2), 0.0) / s2.size();
		s2p = s2p>0?s2p:0.1;
		float s3p = std::accumulate(std::begin(s3)+5, std::end(s3), 0.0) / (s3.size()-5.0);
		s3p = s3p>0?s3p:0.1;
		float s4p = std::accumulate(std::begin(s4), std::begin(s4), 0.0) / s4.size();
		s4p = s4p>0?s4p:0.1;
		float s5p = std::accumulate(std::begin(s5), std::end(s5)-s5.size()/2.0, 0.0) / s5.size()*2.0;
		s5p = s5p>0?s5p:0.1;
		float p1p = std::accumulate(std::begin(p1), std::end(p1), 0.0) / p1.size();
		float atw = std::accumulate(std::begin(atwv), std::end(atwv), 0.0) / atwv.size();
		double p6 = stod(Utility::Instance().config["p6"]);
		double p7 = stod(Utility::Instance().config["p7"]);
		double p8 = stod(Utility::Instance().config["p8"]);
		double p9 = stod(Utility::Instance().config["p9"]);
		double p10 = stod(Utility::Instance().config["p10"]);
		state.push_back(std::array<double,10>{p6,p7,p8,p9,atw,initV,initTheta,0,0,0});

		s1.clear();
		s2.clear();
		s3.clear();
		s4.clear();
		s5.clear();
		p1.clear();
		double xp1 = p1p;
		xp1=xp1<5?xp1:5;
		double xp2 = s5p;
		xp2=xp2<50?xp2:50;
		double xp3 = s4p*p10;
		xp3=xp3<15000?xp3:15000;
		double xp4 = s1p*s5p;
		xp4=xp4<15000?xp4:15000;
		double xp5 = s3p;
		xp5=xp5<15000?xp5:15000;
		double xp11 = (p9*s2p+2*s5p)/s3p;
		xp11=xp11<10?xp11:10;
		std::vector<double> x{xp1,xp2,xp3,xp4,xp5,p10,xp11};
		//std::vector<double> x{0.03,40,6000,200,1500,400,0.2};

		//Regression step for parameters
		nlopt::opt opt(nlopt::LN_COBYLA, 7);
		opt.set_min_objective(costFunction, (void*)&state);

		std::vector<double> lb(7,0.0000001);
		opt.set_lower_bounds(lb);
		std::vector<double> ub(7,15000.0);
		ub[0] = 5;
		ub[1] = 50;
		ub[6] = 10;
		opt.set_upper_bounds(ub);
		opt.set_xtol_rel(1e-6);
		publishLOG("Approximate Values : {" + std::to_string(xp1) + "," + std::to_string(xp2) + "," + std::to_string(xp3) + "," + std::to_string(xp4) + "," + std::to_string(xp5) + "," + std::to_string(p10) + "," + std::to_string(xp11) + "}" );

		double minf;
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
		state.clear();
		exit(0);
	}
	return cmd;
}
