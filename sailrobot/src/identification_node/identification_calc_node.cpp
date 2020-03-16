#include <glm/glm.hpp>
#include <nlopt.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <utilities.hpp>
#include <ros/package.h>
#include <nlopt.hpp>
#include <math.h>
#include <numeric>
#include <chrono>
#include <ctime>

using namespace glm;

double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *option){
	std::vector<std::array<double,12>> state = *(std::vector<std::array<double,12>>*)option;
	double p6 = state.back()[0];
	double p7 = state.back()[1];
	double p8 = state.back()[2];
	double p9 = state.back()[3];
	double atw = state.back()[4];
	double initV = state.back()[5];
	double initTheta = state.back()[6];
	double dtw = state.back()[7];
	double initXRef = state[0][0];
	double initYRef = state[0][1];
	//double dtw = 0.0;
	double dt = 0.001;
	//Reference State gps.x gps.y heading v w dtw aaw daw rud sail
	//TODO

	//Simulate State
	std::vector<std::array<double,5>> predicted;
	std::array<double,5> states{0,0,initTheta,initV,0};
	std::array<double,5> statesData{0,0,0,0,0};
	for(int i = 0; i< state.size()-1; ++i){
		double daw = state[i][7];
		//double dtw = state[i][5];
		double aaw = state[i][6];
		double sigma = cos(daw)+cos(state[i][9]);
		double delta_s = 0;
		if (sigma<0){
			delta_s = M_PI + daw;
		}
		else{
			delta_s = -sign(sin(daw))*state[i][9];
		}

		for(int j = 0; j < 100; ++j){
			double v = states[3];
			double gs = x[3]*aaw*sin(delta_s-daw);
			double gr = x[4]*v*v*sin(state[i][8]);

			states[0] += (v*cos(states[2])+atw*x[0]*cos(dtw))*dt;
			states[1] += (v*sin(states[2])+atw*x[0]*sin(dtw))*dt;
			states[2] += states[4]*dt;
			states[3] += (gs*sin(delta_s)-gr*x[6]*sin(state[i][8])-x[1]*v*v)/p9*dt;
			states[4] += (gs*(p6-p7*cos(delta_s))-gr*p8*cos(state[i][8])-x[2]*states[4]*v)/x[5]*dt;
			//states[2]=mod(states[2],2*M_PI);
		}

		//double latpi = states[0]/(111132.92)*M_PI/180.0;
		//statesData = {states[0]/(111132.92), -states[1]/(111412.84*cos(latpi)-93.5*cos(3*latpi)+0.118*cos(5*latpi)), states[2], states[3], states[4]};
		statesData = {states[0], states[1], states[2], states[3], states[4]};
		predicted.push_back(statesData);
	}

	//cost
	double f = 0;
	int it_i = 0;
	for(std::vector<std::array<double,12>>::iterator it=state.begin(); it != state.end()-11; it++){
		double iGps = (it_i%10)/10.0;
		double xGps = (1.0-iGps)*(*it)[0] + iGps*(*(it+10))[0];
		double yGps = (1.0-iGps)*(*it)[1] + iGps*(*(it+10))[1];

		double gpsdist = Utility::GPSDist(xGps, yGps, initXRef, initYRef);
		//double gpsdist2 = Utility::GPSDist(predicted[it_i][0], predicted[it_i][1], 0, 0);
		double bearing = Utility::GPSBearing(initXRef,initYRef,xGps,yGps);
		//double bearing2 = Utility::GPSBearing(0,0,predicted[it_i][0],predicted[it_i][1]);
		double xpos = gpsdist*cos(bearing);
		double ypos = gpsdist*sin(bearing);
		//double xpos2 = gpsdist2*cos(bearing2);
		//double ypos2 = gpsdist2*sin(bearing2);

		//double xpos = ((*it)[0]);
		double xpos2 = predicted[it_i][0];
		double ypos2 = predicted[it_i][1];
		f += 1*(xpos - xpos2)*(xpos - xpos2);
		f += 1*(ypos - ypos2)*(ypos - ypos2);
		f += 1.0*(sin((*it)[2] - predicted[it_i][2])*sin((*it)[2] - predicted[it_i][2]));
		f += 1.0*((*it)[3] - predicted[it_i][3])*((*it)[3] - predicted[it_i][3]);
		f += 0.0*((*it)[4] - predicted[it_i][4])*((*it)[4] - predicted[it_i][4]);
		//std::cout << (*it)[2]<< "," << predicted[it_i][2] << " " <<ypos << "," <<  predicted[it_i][1] << std::endl;
		//std::cout << "x: " << xpos << " " << xpos2 << std::endl;
		//double ypos = ((*it)[1]);
		//std::cout << "y: " << ypos << " " << ypos2 << std::endl;
		//std::cout << "theta: " << (*it)[2] << " " << predicted[it_i][2] << std::endl;
		/*std::cout << "v: " << (*it)[3] << " " << predicted[it_i][3] << std::endl;
		  std::cout << "w: " << (*it)[4] << " " << predicted[it_i][4] << std::endl;
		  std::cout << std::endl;*/
		it_i++;
	}
	//exit(0);
	return f;
}

int main(int argc, char **argv)
{
	nlopt::algorithm algo = nlopt::LN_BOBYQA;
	if(argc > 1){
		if(!strcmp(argv[argc-1],"-h")){
			/*      std::cout << "\
				NLOPT_GN_DIRECT = 0\n\
				NLOPT_GN_DIRECT_L = 1\n\
				NLOPT_GN_DIRECT_L_RAND = 2\n\
				NLOPT_GN_DIRECT_NOSCAL = 3\n\
				NLOPT_GN_DIRECT_L_NOSCAL = 4\n\
				NLOPT_GN_DIRECT_L_RAND_NOSCAL = 5\n\
				NLOPT_GN_ORIG_DIRECT = 6,\n\
				NLOPT_GN_ORIG_DIRECT_L = 7,\n\
				NLOPT_GD_STOGO = 8,\n\
				NLOPT_GD_STOGO_RAND = 9,\n\
				NLOPT_LD_LBFGS_NOCEDAL = 10,\n\
				NLOPT_LD_LBFGS = 11,\n\
				NLOPT_LN_PRAXIS = 12,\n\
				NLOPT_LD_VAR1 = 13,\n\
				NLOPT_LD_VAR2 = 14,\n\
				NLOPT_LD_TNEWTON = 15,\n\
				NLOPT_LD_TNEWTON_RESTART = 16,\n\
				NLOPT_LD_TNEWTON_PRECOND = 17,\n\
				NLOPT_LD_TNEWTON_PRECOND_RESTART = 18,\n\
				NLOPT_GN_CRS2_LM = 19,\n\
				NLOPT_GN_MLSL = 20,\n\
				NLOPT_GD_MLSL = 21,\n\
				NLOPT_GN_MLSL_LDS = 22,\n\
				NLOPT_GD_MLSL_LDS = 23,\n\
				NLOPT_LD_MMA = 24,\n\
				NLOPT_LN_COBYLA = 25,\n\
				NLOPT_LN_NEWUOA = 26,\n\
				NLOPT_LN_NEWUOA_BOUND = 27,\n\
				NLOPT_LN_NELDERMEAD = 28,\n\
				NLOPT_LN_SBPLX = 29,\n\
				NLOPT_LN_AUGLAG = 30,\n\
				NLOPT_LD_AUGLAG = 31,\n\
				NLOPT_LN_AUGLAG_EQ = 32,\n\
				NLOPT_LD_AUGLAG_EQ = 33,\n\
				NLOPT_LN_BOBYQA = 34,\n\
				NLOPT_GN_ISRES = 35,\n\
				NLOPT_AUGLAG = 36,\n\
				NLOPT_AUGLAG_EQ = 37,\n\
				NLOPT_G_MLSL = 38,\n\
				NLOPT_G_MLSL_LDS = 39,\n\
				NLOPT_LD_SLSQP = 40,\n\
				NLOPT_LD_CCSAQ = 41,\n\
				NLOPT_GN_ESCH = 42,\n\
				NLOPT_GN_AGS = 43,\n\
				NLOPT_NUM_ALGORITHMS = 44\n" << std::endl;*/
			std::cout << "[0:default] LN_BOBYQA,\n\
				[1] LN_NEWUOA_BOUND,\n\
				[2] LN_PRAXIS,\n\
					[3] LN_NELDERMEAD,\n\
						[4] LN_COBYLA,\n" << std::endl;
			exit(0);
		}else
			switch(std::stoi(argv[argc-1])){
				case 0:
					algo = nlopt::LN_BOBYQA;
					break;
				case 1:
					algo = nlopt::LN_NEWUOA_BOUND;
					break;
				case 2:
					algo = nlopt::LN_PRAXIS;
					break;
				case 3:
					algo = nlopt::LN_NELDERMEAD;
					break;
				case 4:
					algo = nlopt::LN_COBYLA;
					break;
				default:
					algo = nlopt::LN_BOBYQA;
					break;
			}
	}

	std::cout << "Begin Identification" << std::endl;

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
	std::vector<double> p1(1,0.03);
	std::vector<double> atwv;
	std::vector<double> dtwv;
	double maxv = 0;
	double prevheading = 0;
	double prevprevheading = 0;

	std::string configPath = "config/config.txt";
	Utility::Instance().config = Utility::ReadConfig(configPath);
	double p10 = stod(Utility::Instance().config["p10"]);
	double p3 = stod(Utility::Instance().config["p3"]);
	double p8 = stod(Utility::Instance().config["p8"]);
	double p9 = stod(Utility::Instance().config["p9"]);
	double p6 = stod(Utility::Instance().config["p6"]);
	double p7 = stod(Utility::Instance().config["p7"]);

	std::getline(infile,line);
	while(std::getline(infile,line)){
		std::stringstream ss(line);

		for (double i; ss >> i;) {
			datain.push_back(i);
			if (ss.peek() == ',')
				ss.ignore();
		}

		double aaw = datain[15];
		double atw = datain[14];
		double daw = datain[16];
		double dtw = datain[13];
		double angacc = 0;
		double angvel = datain[11];
		double heading = datain[12];
		//if(prevprevheading != 0)
		//  angvel = (heading - prevprevheading)/0.01;
		//prevprevheading = heading;
		if(prevheading != 0)
			angacc = (angvel-prevheading)/0.01;
		prevheading = angvel;

		double vnorm = sqrt(datain[3]*datain[3]+datain[4]*datain[4]);
		if(vnorm == 0)
			vnorm = 0.01;
		double anorm = sqrt(datain[6]*datain[6]+datain[7]*datain[7]);
		//double anorm = datain[7];

		switch((int)datain[2]){
			case 0://S2 p1
				if(vnorm > maxv)
					maxv = vnorm;
				s1.push_back((aaw*sin(datain[18]-daw)*sin(datain[18])));
				break;
			case 1:
				s2.push_back(anorm/(vnorm*vnorm));
				break;
			case 2://S3 S1
				s4.push_back(-angacc/(angvel*vnorm));
				s3.push_back(-2*(p10*angacc+p3*angvel*vnorm)/(vnorm*vnorm*p8));
				break;
			case 3://S4 S5
				s5.push_back(anorm/(vnorm*vnorm)*p9);
				break;
		}
		atwv.push_back(atw);
		dtwv.push_back(dtw);
		datain.clear();
	}
	infile.close();

	std::ifstream infileState(path+"/data/identification_state.csv");
	std::vector<std::array<double,12>> state;
	std::getline(infileState,line);
	while(std::getline(infileState,line)){
		std::stringstream ss(line);

		std::array<double,12> dataState;
		int k = 0;
		for (double i; ss >> i;) {
			dataState[k++] = i;
			if (ss.peek() == ',')
				ss.ignore();
		}
		k = 0;
		state.push_back(dataState);
	}


	for(int i = 0; i < s1.size(); ++i)
		s1[i] = maxv*maxv/s1[i];
	float s1p = std::accumulate(s1.begin()+50, s1.end(), 0.0) / (s1.size()-50.0);
	s1p = s1p>0?s1p:0.01;
	float s2p = std::accumulate(s2.begin(), s2.end(), 0.0) / s2.size();
	s2p = s2p>0?s2p:0.01;
	float s3p = std::accumulate(s3.begin()+s3.size()/2.0, s3.end(), 0.0) / s3.size()*2.0;
	s3p = s3p>0?s3p:0.01;
	float s4p = std::accumulate(s4.begin()+s4.size()/2.0, s4.end(), 0.0) / s4.size()*2.0;
	s4p = s4p>0?s4p:0.01;
	float s5p = std::accumulate(s5.begin()+s5.size()/2.0, s5.end(), 0.0) / s5.size()*2.0;
	std::cout << s1p << " " << s2p << " " << s3p << " " << s4p << " " << s5p << " " << std::endl;
	s5p = s5p>0?s5p:0.01;
	float p1p = std::accumulate(p1.begin(), p1.end(), 0.0) / p1.size();
	float atw = std::accumulate(atwv.begin(), atwv.end(), 0.0) / atwv.size();
	float dtw = std::accumulate(dtwv.begin(), dtwv.end(), 0.0) / dtwv.size();
	state.back()[0] = p6;
	state.back()[1] = p7;
	state.back()[2] = p8;
	state.back()[3] = p9;
	state.back()[4] = atw;
	state.back()[7] = dtw;

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
	//nlopt::opt opt(nlopt::LN_BOBYQA, 7);
	//nlopt::opt opt(nlopt::LN_NEWUOA_BOUND, 7);
	//nlopt::opt opt(nlopt::LN_PRAXIS, 7);
	//nlopt::opt opt(nlopt::LN_NELDERMEAD, 7);

	nlopt::opt opt(algo, 7);
	opt.set_min_objective(costFunction, (void*)&state);

	std::vector<double> lb(7,0.0000001);
	opt.set_lower_bounds(lb);
	std::vector<double> ub(7,15000.0);
	ub[0] = 5;
	ub[1] = 50;
	ub[6] = 10;
	opt.set_upper_bounds(ub);
	opt.set_xtol_rel(1e-8);
	std::cout << "Approximate Values : {" + std::to_string(xp1) + "," + std::to_string(xp2) + "," + std::to_string(xp3) + "," + std::to_string(xp4) + "," + std::to_string(xp5) + "," + std::to_string(p10) + "," + std::to_string(xp11) + "}"  << std::endl;

	std::cout << "Starting Optimization" << std::endl;
	double minf;
	auto timenow = std::chrono::system_clock::now();;
	try{
		nlopt::result result = opt.optimize(x, minf);
		std::cout << "found minimum at f(";
		for(int i = 0; i < 6; ++i)
			std::cout << x[i] << ",";
		std::cout << x[6] << ") = " << std::setprecision(10) << minf << std::endl;
		Utility::Instance().config["est_p1"] = std::to_string(x[0]);
		Utility::Instance().config["est_p2"] = std::to_string(x[1]);
		Utility::Instance().config["est_p3"] = std::to_string(x[2]);
		Utility::Instance().config["est_p4"] = std::to_string(x[3]);
		Utility::Instance().config["est_p5"] = std::to_string(x[4]);
		Utility::Instance().config["est_p10"] = std::to_string(x[5]);
		Utility::Instance().config["est_p11"] = std::to_string(x[6]);
		Utility::Instance().config["vmax"] = std::to_string(maxv);
		Utility::SaveConfig("config/config.txt");
	}
	catch(std::exception &e) {
		std::cout << "nlopt failed: " << e.what() << std::endl;
	}
	state.clear();
	std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-timenow;
	std::cout << "time spent : " << elapsed_seconds.count() << std::endl;
	return 0;
}
