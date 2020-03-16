#include "mpc_node/mpc.hpp"
#include <math.h>
#include <fstream>
#include <iostream>
#include <utilities.hpp>
#include <ros/package.h>


using namespace Sailboat;
using namespace glm;

void MPC::setup(ros::NodeHandle* n){
	std::string goalPath = "data/goalpoint.txt";
	int algoN = 0;
	n->param<std::string>("goal",goalPath,goalPath);
	n->param<int>("receding", receding_n, 10);
	n->param<int>("algo", algoN, 0);

	switch(algoN){
		case 0:
		algo = nlopt::LN_BOBYQA;
		std::cout << "Chose BOBYQA" << std::endl;
		break;
		case 1:
		algo = nlopt::LN_NEWUOA_BOUND;
		std::cout << "Chose NEWUOA_BOUND" << std::endl;
		break;
		case 2:
		algo = nlopt::LN_PRAXIS;
		std::cout << "Chose PRAXIS" << std::endl;
		break;
		case 3:
		algo = nlopt::LN_NELDERMEAD;
		std::cout << "Chose NELDERMEAD" << std::endl;
		break;
		case 4:
		algo = nlopt::LN_COBYLA;
		std::cout << "Chose COBYLA" << std::endl;
		break;
		default:
		algo = nlopt::LN_BOBYQA;
		std::cout << "Chose BOBYQA" << std::endl;
		break;
	}

	waypoints.push_back(dvec2(gpsMsg.latitude, gpsMsg.longitude));
	waypoints = Utility::AppendGPSCoordinates(goalPath, nbWaypoints, &waypoints);
	if(nbWaypoints == 0){
		std::cerr << "Goals empty" << std::endl;
		exit(0);
	}
	initXRef = gpsMsg.latitude;
	initYRef = gpsMsg.longitude;
	currentWaypoint = 0;

  std::string configPath = "config/config.txt";
  Utility::Instance().config = Utility::ReadConfig(configPath);

	inputs_n = receding_n/step*2;
	lb = std::vector<double>(inputs_n,-M_PI/4.0);
	for(int i = 1; i < inputs_n; i+=2)
		lb[i] = -M_PI/2.0;
		//lb[i] = 0;
	ub = std::vector<double>(inputs_n,M_PI/4.0);
	for(int i = 1; i < inputs_n; i+=2)
		ub[i] = M_PI/2.0;
}

double MPC::costFunction(const std::vector<double> &x, std::vector<double> &grad, void *option){

	unsigned int n = x.size();
	double* optionD = (double*) option;
	//Parameters
	unsigned int receding_n = optionD[0];
	int step = optionD[1];
	int receding_n5 = 9;

	//Reference State
	//TODO

	//Actual State
	double state[5];
	memcpy(state,optionD+receding_n5,5*sizeof(double));

	//MPC parameters
	double dt = optionD[receding_n5+5];
	double aaw = optionD[receding_n5+6];
	double daw = optionD[receding_n5+7];
	double atw = optionD[receding_n5+8];
	double dtw = optionD[receding_n5+9];
	double pconfig[11];
	memcpy(pconfig,optionD+receding_n5+10,11*sizeof(double));
	double Q[5];
	memcpy(Q,optionD+receding_n5+21,5*sizeof(double));

	//prediction
	std::vector<double*> state_predict;
	for(int i = 0; i < receding_n; ++i){
		int input_i = (2*i/step);
    /*double sigma = cos(daw)+cos(x[input_i+1]);
    double delta_s = 0;
    if (sigma<0){
      delta_s = M_PI + daw;
    }
    else{
      delta_s = -sign(sin(daw))*x[input_i+1];
    }*/
		double delta_s = x[input_i+1];
		for(int j = 0; j < 10; ++j){
			double v = state[3];
			double gs = pconfig[3]*aaw*sin(delta_s-daw);
			double gr = pconfig[4]*v*v*sin(x[input_i]);

			state[0] += (v*cos(state[2])+atw*pconfig[0]*cos(dtw))*dt;
			state[1] += (v*sin(state[2])+atw*pconfig[0]*sin(dtw))*dt;
			state[2] += state[4]*dt;
			state[3] += (gs*sin(delta_s)-gr*pconfig[10]*sin(x[input_i])-pconfig[1]*v*v)/pconfig[8]*dt;
			state[4] += (gs*(pconfig[5]-pconfig[6]*cos(delta_s))-gr*pconfig[7]*cos(x[input_i])-pconfig[2]*state[4]*v)/pconfig[9]*dt;
		}
		double states[5];
		memcpy(states, state, 5*sizeof(double));
		state_predict.push_back(states);
	}

	//cost
	double r = 10.0;
	double f = 0;
	int it_i = 2;
	double normx = abs(optionD[2] - optionD[7]);
	double normy = abs(optionD[3] - optionD[8]);
	for(std::vector<double*>::iterator it=state_predict.begin(); it != state_predict.end(); it++){
		f += Q[0]*((*it)[0] - optionD[2])*((*it)[0] - optionD[2])/(normx*normx);
		f += Q[1]*((*it)[1] - optionD[3])*((*it)[1] - optionD[3])/(normy*normy);
		f += Q[2]*((*it)[2] - optionD[4])*((*it)[2] - optionD[4]);
		f += Q[3]*((*it)[3] - optionD[5])*((*it)[3] - optionD[5])/(optionD[5]*optionD[5]);
		//f += Q[4]*((*it)[4] - (optionD)[6])*((*it)[4] - (optionD)[6]);

		//TackingStrategy
		double nogo = 0.0;
		double tacking_phi = M_PI/4.0;


		dvec2 toWaypoints = dvec2(optionD[2] - optionD[7],optionD[3] - optionD[8]);
		toWaypoints /= glm::length(toWaypoints);
		dvec2 distWaypoints = dvec2(optionD[receding_n5] - optionD[7], optionD[receding_n5+1] - optionD[8]);
		double e = toWaypoints.x*distWaypoints.y-toWaypoints.y*distWaypoints.x;


		if(cos(dtw-(*it)[2])+cos(tacking_phi)<0.0){
			if(abs(e) < r/2.0){
				nogo = abs(cos(dtw-(*it)[2])+cos(tacking_phi));
			}else{
				nogo = abs(cos(dtw-(*it)[2])+cos(tacking_phi))/2.0;
			}
		}
		f += nogo*nogo;

		it_i += 5;
	}



	/*if(grad.size()>0){
		for(int i = 0; i < n; i+=2){
			grad[i] = 0;
			grad[i+1] = 0;
			for(int j = 0; j < step; ++j){
				int i_j = i/2*step+j;
				grad[i] += (2*Q[3]*((state_predict[i_j])[3] - ((double*)option)[i_j*5+3])*((state_predict[i_j])[3] - ((double*)option)[i+3]))*
					(-2*pconfig[10]*pconfig[4]/pconfig[8]*(state_predict[i_j])[3]*(state_predict[i_j])[3]*sin(x[i_j])*cos(x[i_j]));
				grad[i] += (2*Q[4]*((state_predict[i_j])[4] - ((double*)option)[i_j*5+4])*((state_predict[i_j])[4] - ((double*)option)[i_j*5+4]))*
					(-pconfig[4]*pconfig[7]/pconfig[9]*(1-2*sin(x[i_j])*sin(x[i_j])));

				grad[i+1] += (2*Q[3]*((state_predict[i_j])[3] - ((double*)option)[i_j*5+3])*((state_predict[i_j])[3] - ((double*)option)[i_j*5+3]))*
					(pconfig[3]/pconfig[8]*sin(2*x[i_j+1]-daw)*aaw);
				grad[i+1] += (2*Q[4]*((state_predict[i_j])[4] - ((double*)option)[i_j*5+4])*((state_predict[i_j])[4] - ((double*)option)[i_j*5+4]))*
					(pconfig[3]/pconfig[9]*aaw*(pconfig[5]*cos(x[i_j]-daw)-pconfig[6]*cos(2*x[i_j]-daw)));
			}
		}
	}*/
	state_predict.clear();
	return f;
}

double MPC::constraintFunction(unsigned n, const double *x, double *grad, void*data){

}

geometry_msgs::Twist MPC::control(){
	geometry_msgs::Twist cmd;
	dvec2 current(gpsMsg.latitude, gpsMsg.longitude);
	float currentHeading = (Utility::QuaternionToEuler(imuMsg.orientation)).z;

	double vnorm = sqrt(velMsg.linear.x*velMsg.linear.x+velMsg.linear.y*velMsg.linear.y);
	float windNorthA = 0.0;
	float windNorth = Utility::RelativeToTrueWind(dvec2(velMsg.linear.x,velMsg.linear.y),currentHeading,windMsg.theta, windMsg.x, windMsg.y, &windNorthA);

	/// Calculate the distance to the next waypoint, if close, change to the next waypoint
	float dist = Utility::GPSDist(current, waypoints[(currentWaypoint+1)%nbWaypoints]);
	if(dist < 10){
		publishLOG("PArrived at waypoint " + std::to_string((currentWaypoint+1)%nbWaypoints));
		if(waypoints.size() > nbWaypoints)
			waypoints.erase(waypoints.begin());
		else{
			currentWaypoint++;
			currentWaypoint %= nbWaypoints;
		}
	}

	//int receding_n5 = receding_n*5+2;
	int receding_n5 = 9;
	double optionData[receding_n5+26];
	//Parameters
	optionData[0] = receding_n;
	optionData[1] = step; //input step
	//Rerence State
	double gpsdist = Utility::GPSDist(waypoints[(currentWaypoint+1)%nbWaypoints].x,waypoints[(currentWaypoint+1)%nbWaypoints].y, initXRef, initYRef);
  double bearing = Utility::GPSBearing(initXRef,initYRef,waypoints[(currentWaypoint+1)%nbWaypoints].x,waypoints[(currentWaypoint+1)%nbWaypoints].y);
  double bearingBoat = Utility::GPSBearing(gpsMsg.latitude,gpsMsg.longitude,waypoints[(currentWaypoint+1)%nbWaypoints].x,waypoints[(currentWaypoint+1)%nbWaypoints].y);
  double xpos = gpsdist*cos(bearing);
  double ypos = gpsdist*sin(bearing);
	optionData[2] = xpos; //x
	optionData[3] = ypos; //y
	optionData[4] = bearingBoat; //theta
	optionData[5] = stod(Utility::Instance().config["vmax"]); //v
	optionData[6] = 0; //w

	gpsdist = Utility::GPSDist(waypoints[currentWaypoint].x,waypoints[currentWaypoint].y, initXRef, initYRef);
	bearing = Utility::GPSBearing(initXRef,initYRef,waypoints[currentWaypoint].x,waypoints[currentWaypoint].y);
  xpos = gpsdist*cos(bearing);
  ypos = gpsdist*sin(bearing);
	optionData[7] = xpos;
	optionData[8] = ypos;

	//Actual State
	gpsdist = Utility::GPSDist(gpsMsg.latitude, gpsMsg.longitude, initXRef, initYRef);
  bearing = Utility::GPSBearing(initXRef,initYRef,gpsMsg.latitude, gpsMsg.longitude);
  xpos = gpsdist*cos(bearing);
  ypos = gpsdist*sin(bearing);
	optionData[receding_n5] = xpos; //x
	optionData[receding_n5+1] = ypos; //y
	optionData[receding_n5+2] = currentHeading; //theta
	optionData[receding_n5+3] = vnorm; //v
	optionData[receding_n5+4] = imuMsg.angular_velocity.z; //w

	//Data for MPC
	optionData[receding_n5+5] = 0.001; //dt
	optionData[receding_n5+6] = sqrt(windMsg.x*windMsg.x+windMsg.y*windMsg.y); //aaw
	optionData[receding_n5+7] = windMsg.theta; //daw
	optionData[receding_n5+8] = windNorthA; //atw;
	optionData[receding_n5+9] = windNorth; //dtw
	double pconfig[11] = {
		stod(Utility::Instance().config["est_p1"]),
		stod(Utility::Instance().config["est_p2"]),
		stod(Utility::Instance().config["est_p3"]),
		stod(Utility::Instance().config["est_p4"]),
		stod(Utility::Instance().config["est_p5"]),
		stod(Utility::Instance().config["p6"]),
		stod(Utility::Instance().config["p7"]),
		stod(Utility::Instance().config["p8"]),
		stod(Utility::Instance().config["p9"]),
		stod(Utility::Instance().config["p10"]),
		stod(Utility::Instance().config["est_p11"])
	};
	memcpy(optionData+receding_n5+10,pconfig,11*sizeof(double)); //parameters
	double Q[5] = {1.0,1.0,0.1,0.2,0.0};
	memcpy(optionData+receding_n5+21,Q,5*sizeof(double)); //Gain


	//nlopt::opt opt(nlopt::LN_BOBYQA, inputs_n);
	//nlopt::opt opt(nlopt::LN_NEWUOA_BOUND, inputs_n);
	//nlopt::opt opt(nlopt::LN_PRAXIS, inputs_n);
	//nlopt::opt opt(nlopt::LN_NELDERMEAD, inputs_n);
	nlopt::opt opt(algo, inputs_n);

	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);
	opt.set_min_objective(costFunction, (void*)optionData);
	//my_constraint_data data[2] = { {2,0}, {-1,1} };
	//opt.add_inequality_constraint(myconstraint, &data[0], 1e-8);
	//opt.add_inequality_constraint(myconstraint, &data[1], 1e-8);
	opt.set_xtol_rel(1e-5);
	//opt.set_maxeval(20000);
	opt.set_maxtime(3);
	std::vector<double> x(inputs_n,rudderAngle);
	for(int i = 1; i < inputs_n; i+=2)
		x[i] = sailAngle;

	double minf;

	try{
    		nlopt::result result = opt.optimize(x, minf);
    		std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "
        	<< std::setprecision(10) << minf << std::endl;
	}
	catch(std::exception &e) {
    		std::cout << "nlopt failed: " << e.what() << std::endl;
	}


	cmd.angular.x = x[0];
	cmd.angular.y = sign(x[1])*x[1];

	publishMSG("MPC Controlling");
	publishMarkerA(waypoints[currentWaypoint].x,waypoints[currentWaypoint].y);
	publishMarkerB(waypoints[(currentWaypoint+1)%nbWaypoints].x,waypoints[(currentWaypoint+1)%nbWaypoints].y);

	x.clear();
	return cmd;
}
