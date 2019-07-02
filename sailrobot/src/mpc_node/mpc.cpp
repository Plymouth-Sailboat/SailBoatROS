#include "mpc_node/mpc.hpp"
#include <math.h>
#include <fstream>
#include <iostream>
#include <utilities.hpp>
#include <ros/package.h>

#include <nlopt.hpp>

using namespace Sailboat;
using namespace glm;

void MPC::setup(ros::NodeHandle* n){
	std::string goalPath = "data/goalpoint.txt";
	if (n->hasParam("goal"))
		n->getParam("goal",goalPath);

	waypoints = Utility::ReadGPSCoordinates(goalPath, nbWaypoints);
	if(waypoints == NULL){
		std::cerr << "Goals empty" << std::endl;
		exit(0);
	}

	if(nbWaypoints < 1){
		std::cerr << "No Waypoints" << std::endl;
		exit(0);
	}
}

double MPC::costFunction(unsigned n, const double *x, double *grad, void *option){

	double state[5];
	unsigned int receding_n = ((double*)option)[n/2*5+5];
	double dt = ((double*)option)[receding_n*5+6];
	double aaw = ((double*)option)[receding_n*5+7];
	double daw = ((double*)option)[receding_n*5+8];
	double atw = ((double*)option)[receding_n*5+9];
	double dtw = ((double*)option)[receding_n*5+10];
	int step = ((double*)option)[receding_n*5+11];
	double pconfig[11];
	memcpy(pconfig,((double*)option)+receding_n*5+12,11*sizeof(double));
	double Q[5];
	memcpy(Q,((double*)option)+receding_n*5+23,5*sizeof(double));

	for(int i = 0; i < 5; ++i){
		state[i] = ((double*)option)[receding_n*5+i];
	}

	//prediction
	std::vector<double*> state_predict;
	for(int i = 0; i < receding_n; ++i){
		int input_i = (2*i/step);
		double gs = pconfig[3]*aaw*sin(x[input_i+1]-daw);
		double gr = pconfig[4]*state[3]*state[3]*sin(x[input_i]);

		state[0] += state[4]*cos(state[3])+aaw*pconfig[0]*cos(dtw)*dt;
		state[1] += state[4]*sin(state[3])+aaw*pconfig[0]*sin(dtw)*dt;
		state[2] += state[5]*dt;
		state[3] += (gs*sin(x[input_i+1])-gr*pconfig[10]*sin(x[input_i])-pconfig[1]*state[3]*state[3])/pconfig[8]*dt;
		state[4] += (gs*(pconfig[6]-pconfig[7]*cos(x[input_i+1]))-gr*pconfig[7]*cos(x[input_i])-pconfig[2]*state[4]*state[3])/pconfig[9]*dt;
		double* states = new double[5];
		memcpy(states, state, 5*sizeof(double));
		state_predict.push_back(states);
	}
	
	//cost
	double f = 0;
	int it_i = 0;
	for(std::vector<double*>::iterator it=state_predict.begin(); it != state_predict.end(); it++){
	f += Q[0]*((*it)[0] - ((double*)option)[it_i])*((*it)[0] - ((double*)option)[it_i]);
	f += Q[1]*((*it)[1] - ((double*)option)[it_i+1])*((*it)[1] - ((double*)option)[it_i+1]);
	f += Q[2]*((*it)[2] - ((double*)option)[it_i+2])*((*it)[2] - ((double*)option)[it_i+2]);
	f += Q[3]*((*it)[3] - ((double*)option)[it_i+3])*((*it)[3] - ((double*)option)[it_i+3]);
	f += Q[4]*((*it)[4] - ((double*)option)[it_i+4])*((*it)[4] - ((double*)option)[it_i+4]);
	it_i += 5;
	}
	if(grad){
		for(int i = 0; i < n; i+=2){
			grad[i] = 0;
			grad[i+1] = 0;
			for(int j = 0; j < step; ++j){
				int i_j = i*step+j;
				grad[i] += (2*Q[3]*((state_predict[i_j])[3] - ((double*)option)[i_j*5+3])*((state_predict[i_j])[3] - ((double*)option)[i+3]))*
					(-2*pconfig[10]*pconfig[4]/pconfig[9]*(state_predict[i_j])[3]*(state_predict[i_j])[3]*sin(x[i_j])*cos(x[i_j]));
				grad[i] += (2*Q[4]*((state_predict[i_j])[4] - ((double*)option)[i_j*5+4])*((state_predict[i_j])[4] - ((double*)option)[i_j*5+4]))*
					(-pconfig[4]*pconfig[7]/pconfig[9]*(1-2*sin(x[i_j])*sin(x[i_j])));
			
				grad[i+1] += (2*Q[3]*((state_predict[i_j])[3] - ((double*)option)[i_j*5+3])*((state_predict[i_j])[3] - ((double*)option)[i_j*5+3]))* 
					(pconfig[3]/pconfig[8]*sin(2*x[i_j+1]-daw)*aaw);
				grad[i+1] += (2*Q[4]*((state_predict[i_j])[4] - ((double*)option)[i_j*5+4])*((state_predict[i_j])[4] - ((double*)option)[i_j*5+4]))*
					(pconfig[3]/pconfig[9]*aaw*(pconfig[5]*cos(x[i_j]-daw)-pconfig[7]*cos(2*x[i_j]-daw)));
			}
		}
	}
	state_predict.clear();
	return f;
}

double MPC::constraintFunction(unsigned n, const double *x, double *grad, void*data){
	
}

geometry_msgs::Twist MPC::control(){
	geometry_msgs::Twist cmd;
	vec2 current(gpsMsg.latitude, gpsMsg.longitude);
	float currentHeading = (Utility::QuaternionToEuler(imuMsg.orientation)).z;

	vec3 gpsXY = Utility::GPSToCartesian(current);
	float windNorth = windMsg.theta + currentHeading;

	unsigned int receding_n = 20;
	unsigned int step = 1;
	unsigned int inputs_n = receding_n/step*2;

	nlopt::opt opt(nlopt::LD_SLSQP, inputs_n);
	
	std::vector<double> lb(inputs_n,-M_PI/4.0);
	for(int i = 1; i < inputs_n; i+=2)
		lb[i] = 0;
	std::vector<double> ub(inputs_n,M_PI/4.0);
	for(int i = 1; i < inputs_n; i+=2)
		ub[i] = M_PI/2.0;

	opt.set_lower_bounds(lb);
	opt.set_upper_bounds(ub);

	double optionData[receding_n*5+28]; 
	//Actual State
	optionData[receding_n*5] = gpsXY.x; //x
	optionData[receding_n*5] = gpsXY.y; //y
	optionData[receding_n*5] = currentHeading; //theta
	optionData[receding_n*5] = 0.0; //v
	optionData[receding_n*5] = 0.0; //w
	
	//Data for MPC
	optionData[receding_n*5+5] = receding_n;
	optionData[receding_n*5+6] = 0.01; //dt
	optionData[receding_n*5+7] = 1.0; //aaw
	optionData[receding_n*5+8] = windMsg.theta;
	optionData[receding_n*5+9] = 1.0; //atw;
	optionData[receding_n*5+10] = windNorth; //dtw
	optionData[receding_n*5+11] = step; //input step
	double pconfig[11];
	memcpy(optionData+receding_n*5+12,pconfig,11*sizeof(double)); //parameters
	double Q[5] = {1.0,1.0,0.0,0.0,0.0};
	memcpy(optionData+receding_n*5+23,Q,5*sizeof(double)); //Gain


	opt.set_min_objective(costFunction, (void*)optionData);
	//my_constraint_data data[2] = { {2,0}, {-1,1} };
	//opt.add_inequality_constraint(myconstraint, &data[0], 1e-8);
	//opt.add_inequality_constraint(myconstraint, &data[1], 1e-8);
	opt.set_xtol_rel(1e-4);
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
	cmd.angular.y = x[1];

	publishMSG("MPC Controlling");
	return cmd;
}

