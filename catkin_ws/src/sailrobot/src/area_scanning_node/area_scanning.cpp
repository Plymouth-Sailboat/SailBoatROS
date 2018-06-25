#include "area_scanning_node/area_scanning.hpp"
#include "math.h"
#include <ros/package.h>
#include <utilities.hpp>
#include <glm/glm.hpp>
#include <iostream>
#include <fstream>
#include <cstdlib>

using namespace Sailboat;
using namespace glm;

void AreaScanning::setup(ros::NodeHandle* n){
	std::string path = ros::package::getPath("sailrobot");
	
	std::string areaPath = "data/area_scanning.txt";
	if(n->hasParam("area"))
		n->getParam("area", areaPath);

	waypoints = Utility::ReadGPSCoordinates(areaPath, nbWaypoints);
	if(waypoints == NULL){
		std::cerr << "Waypoints Coordinates File not Found" << std::endl;
		exit(0);
	}
	currentWaypoint = 0;

	buildLKHFiles();
	std::system((path + "/bin/LKH " + path + "/bin/LKHData/area_scanning.par").c_str());
	readResults();
}

void AreaScanning::buildLKHFiles(){
	std::string path = ros::package::getPath("sailrobot");
	std::ofstream params(path + "/bin/LKHData/area_scanning.par");
	if(params.is_open()){
		params << "PROBLEM_FILE = " + path + "/bin/LKHData/area_scanning.tsp\n";
		params << "OPTIMUM = 1000 \n"; 
		params << "MOVE_TYPE = 5 \n"; 
		params << "PATCHING_C = 3 \n"; 
		params << "PATCHING_A = 2 \n"; 
		params << "RUNS = 10 \n";
		params << "OUTPUT_TOUR_FILE = " + path + "/bin/LKHData/area_scanning.tour\n"; 
		params.close();
	}else{
		std::cerr << "Couldn't write param file" << std::endl;
		exit(0);
	}

	std::ofstream problem(path + "/bin/LKHData/area_scanning.tsp");
	if(problem.is_open()){
		problem << "NAME : area_scanning\n";
		problem << "COMMENT : Used for Dubins Area Scanning\n";
		problem << "TYPE : ATSP\n";
		problem << "DIMENSION : " << nbWaypoints << " \n";
		problem << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
		problem << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
		problem << "EDGE_WEIGHT_SECTION\n";

		for(int i = 0; i < nbWaypoints; ++i){
			for(int j = 0; j < nbWaypoints-1; ++j){
				int dist = (Utility::GPSDist(waypoints[i], waypoints[j])*10);
				if(dist == 0)
					dist = 9999;
				float bearing = Utility::GPSBearing(waypoints[i], waypoints[j]);
				float wind = windMsg.theta;
				if(cos(bearing - wind) > 0.83)
					dist += 1000;
				problem << dist << " ";
			}
			int dist = (Utility::GPSDist(waypoints[i], waypoints[nbWaypoints-1])*10);
                        if(dist == 0)
                        	dist = 9999;
			problem << dist  << "\n";
		}

		problem << "EOF";
	}else{
		std::cerr << "Couldn't write problem file" << std::endl;
		exit(0);
	}
}

void AreaScanning::readResults(){
	waypointsOrder = new int[nbWaypoints];

        std::string path = ros::package::getPath("sailrobot");
        std::ifstream f(path + "/bin/LKHData/area_scanning.tour");
	if(f.is_open()){
		std::string line;
		int i = 0;
		while (std::getline(f,line)) {
  			if(!line.empty() && isdigit(line[0]))
				waypointsOrder[i++] = std::stoi(line);
		}
		f.close();
		std::string waypointsS = "P";
		for(int i = 0; i < nbWaypoints; ++i){
			waypointsS += std::to_string(waypointsOrder[i]) + "\n";
		}
		publishMSG(waypointsS);
	}else{
		std::cerr << "Couldn't open tour file" << std::endl;
		exit(0);
	}
}

geometry_msgs::Twist AreaScanning::control(){
	geometry_msgs::Twist cmd;

	vec2 current = vec2(gpsMsg.latitude, gpsMsg.longitude);

	if(Utility::GPSDist(current, waypoints[waypointsOrder[currentWaypoint]]) < 10)
		currentWaypoint++;
	if(currentWaypoint > nbWaypoints)
		return cmd;

	float wind = windMsg.theta;
	vec3 heading = Utility::QuaternionToEuler(imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w);
	float theta = Utility::GPSBearing(current, waypoints[waypointsOrder[currentWaypoint]]);

	cmd.angular.z = theta;

	return cmd;
}
