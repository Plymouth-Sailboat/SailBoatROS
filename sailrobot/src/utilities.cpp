#include "utilities.hpp"
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>
#include <algorithm>
#include <ros/package.h>

using namespace glm;

Utility* Utility::instance = nullptr;

float Utility::GPSDist(float lat1, float lon1, float lat2, float lon2){
	float R = 6371000;
	float ksi1 = lat1*M_PI/180.0;
	float ksi2 = lat2*M_PI/180.0;
	float dksi = (lat2-lat1)*M_PI/180.0;
	float dlambda = (lon2-lon1)*M_PI/180.0;

	float a = sin(dksi/2.0)*sin(dksi/2.0) + cos(ksi1)*cos(ksi2)*sin(dlambda/2.0)*sin(dlambda/2.0);
	float c = 2*atan2(sqrt(a), sqrt(1-a));
	return R*c;
}

float Utility::GPSDist(dvec2 point1, dvec2 point2){
	float R = 6371000;
	float ksi1 = point1.x*M_PI/180.0;
	float ksi2 = point2.x*M_PI/180.0;
	float dksi = (point2.x-point1.x)*M_PI/180.0;
	float dlambda = (point2.y-point1.y)*M_PI/180.0;

	float a = sin(dksi/2.0)*sin(dksi/2.0) + cos(ksi1)*cos(ksi2)*sin(dlambda/2.0)*sin(dlambda/2.0);
	float c = 2*atan2(sqrt(a), sqrt(1-a));
	return R*c;
}

float Utility::GPSDistFast(float lat1, float lon1, float lat2, float lon2){
	float ksi1 = lat1*M_PI/180.0;
	float ksi2 = lat2*M_PI/180.0;
	float lam1 = lon1*M_PI/180.0;
	float lam2 = lon2*M_PI/180.0;

	float x = (lam2-lam1)*cos((ksi1+ksi2)/2.0);
	float y = ksi2-ksi1;
	return sqrt(x*x + y*y) * 6371000;
}

float Utility::GPSDistFast(dvec2 point1, dvec2 point2){
	float ksi1 = point1.x*M_PI/180.0;
	float ksi2 = point2.x*M_PI/180.0;
	float lam1 = point1.y*M_PI/180.0;
	float lam2 = point2.y*M_PI/180.0;

	float x = (lam2-lam1)*cos((ksi1+ksi2)/2.0);
	float y = ksi2-ksi1;
	return sqrt(x*x + y*y) * 6371000;
}

float Utility::GPSBearing(float lat1, float lon1, float lat2, float lon2){
	float ksi1 = lat1*M_PI/180.0;
	float ksi2 = lat2*M_PI/180.0;
	float lam1 = lon1*M_PI/180.0;
	float lam2 = lon2*M_PI/180.0;

	float y = sin(lam2-lam1)*cos(ksi2);
	float x = cos(ksi1)*sin(ksi2)-sin(ksi1)*cos(ksi2)*cos(lam2-lam1);
	return -atan2(y,x);
}

float Utility::GPSBearing(dvec2 point1, dvec2 point2){
	float ksi1 = point1.x*M_PI/180.0;
	float ksi2 = point2.x*M_PI/180.0;
	float lam1 = point1.y*M_PI/180.0;
	float lam2 = point2.y*M_PI/180.0;

	float y = sin(lam2-lam1)*cos(ksi2);
	float x = cos(ksi1)*sin(ksi2)-sin(ksi1)*cos(ksi2)*cos(lam2-lam1);
	return -atan2(y,x);
}

glm::dvec3 Utility::GPSToCartesian(float lat, float gpslong){
	float ksi1 = lat*M_PI/180.0;
	float lam1 = gpslong*M_PI/180.0;
	return glm::dvec3(6371000*cos(ksi1)*cos(lam1),
			6371000*cos(ksi1)*sin(lam1),
			6371000*sin(ksi1));
}

glm::dvec3 Utility::GPSToCartesian(glm::dvec2 gpsposition){
	return GPSToCartesian((float)gpsposition.x,(float)gpsposition.y);
}

float Utility::CartesianToGPS(float x, float y){

}

dvec3 Utility::QuaternionToEuler(geometry_msgs::Quaternion q){
	return eulerAngles(quat(q.w, q.x, q.y, q.z));
}

dvec3 Utility::QuaternionToEuler(quat q){
	return eulerAngles(q);
}

dvec3 Utility::QuaternionToEuler(float x, float y, float z, float w){
	return eulerAngles(quat(w,x,y,z));
}

quat Utility::EulerToQuaternion(dvec3 v){
	return quat(v);
}

quat Utility::EulerToQuaternion(float x, float y, float z){
	return quat(dvec3(x,y,z));
}

dvec2* Utility::ReadGPSCoordinates(std::string filepath, int& size){
	dvec2* coordinates = NULL;
	std::string path = ros::package::getPath("sailrobot");
	if(filepath[0] != '/')
		filepath = path + "/" + filepath;
	std::cout << "Reading GPS coordinates from the file " << filepath << std::endl;
	std::fstream file(filepath);
	if(!file){
		std::cerr << "GPS Reading : Couldn't open file" << std::endl;
		return coordinates;
	}
	std::string line;
	int nbLines = std::count(std::istreambuf_iterator<char>(file),
			std::istreambuf_iterator<char>(), '\n');
	size = nbLines;
	coordinates = new dvec2[nbLines];
	file.clear();
	file.seekg(0, std::ios::beg);
	int i = 0;
	while (std::getline(file, line)) {
		std::string coords;
		std::stringstream stream(line);
		std::getline(stream, coords, ',');
		coordinates[i].x = std::atof(coords.c_str());
		std::getline(stream, coords, ',');
		coordinates[i].y = std::atof(coords.c_str());
		i++;
	}
	file.close();
	return coordinates;
}


dvec2* Utility::AppendGPSCoordinates(std::string filepath, int& size, glm::dvec2* list, int sizeList){
	dvec2* coordinates = NULL;
	std::string path = ros::package::getPath("sailrobot");
	if(filepath[0] != '/')
		filepath = path + "/" + filepath;
	std::cout << "Reading GPS coordinates from the file " << filepath << std::endl;
	std::fstream file(filepath);
	if(!file){
		std::cerr << "GPS Reading : Couldn't open file" << std::endl;
		return coordinates;
	}
	std::string line;
	int nbLines = std::count(std::istreambuf_iterator<char>(file),
			std::istreambuf_iterator<char>(), '\n');
	size = nbLines;
	coordinates = new dvec2[nbLines+sizeList];
	for(int i = 0; i < sizeList; ++i)
		coordinates[i] = list[i];
	file.clear();
	file.seekg(0, std::ios::beg);
	int i = 0;
	while (std::getline(file, line)) {
		std::string coords;
		std::stringstream stream(line);
		std::getline(stream, coords, ',');
		coordinates[i+sizeList].x = std::atof(coords.c_str());
		std::getline(stream, coords, ',');
		coordinates[i+sizeList].y = std::atof(coords.c_str());
		i++;
	}
	file.close();
	return coordinates;
}

std::vector<dvec2> Utility::AppendGPSCoordinates(std::string filepath, int& size, std::vector<glm::dvec2>* list){
	std::string path = ros::package::getPath("sailrobot");
	if(filepath[0] != '/')
		filepath = path + "/" + filepath;
	std::cout << "Reading GPS coordinates from the file " << filepath << std::endl;
	std::fstream file(filepath);
	if(!file){
		std::cerr << "GPS Reading : Couldn't open file" << std::endl;
		return *list;
	}
	std::string line;
	int nbLines = std::count(std::istreambuf_iterator<char>(file),
			std::istreambuf_iterator<char>(), '\n');
	size = nbLines;
	file.clear();
	file.seekg(0, std::ios::beg);
	int i = 0;
	while (std::getline(file, line)) {
		std::string coords;
		std::stringstream stream(line);
		std::getline(stream, coords, ',');
		float x = std::atof(coords.c_str());
		std::getline(stream, coords, ',');
		float y = std::atof(coords.c_str());
		list->push_back(glm::dvec2(x,y));
		i++;
	}
	file.close();
	return *list;
}

std::map<std::string,std::string> Utility::ReadConfig(std::string filepath){
	std::map<std::string,std::string> res;

	std::string path = ros::package::getPath("sailrobot");
	if(filepath[0] != '/')
		filepath = path + "/" + filepath;
	std::cout << "Reading boat configuration from the file " << filepath << std::endl;
	std::fstream file(filepath);
	if(!file){
		std::cerr << "Config Reading : Couldn't open file" << std::endl;
		return res;
	}
	std::string line;
	while (std::getline(file, line)) {
		std::istringstream is_line(line);
		std::string key;
		if( std::getline(is_line, key, '=') )
		{
			std::string value;
			if( std::getline(is_line, value) ){
				std::string val = value.substr(0,value.find(" "));
				res[key]=val.substr(0,val.find("/"));
			}
		}
	}
	file.close();
	return res;
}

void Utility::SaveConfig(std::string filepath){
	std::string path = ros::package::getPath("sailrobot");
	if(filepath[0] != '/')
		filepath = path + "/" + filepath;
	std::ofstream file(filepath, std::ios::out | std::ios::trunc);
	if(!file){
		std::cerr << "Config Writing : Couldn't open file" << std::endl;
		return;
	}
	std::string line;
	for(std::map<std::string,std::string>::iterator it = Utility::Instance().config.begin(); it != Utility::Instance().config.end(); it++)
		file << it->first << "=" << it->second << std::endl;
	file.close();
}

float Utility::RelativeToTrueWind(glm::dvec2 v, float heading, float windDirection, float windAccx, float windAccy, float* windNorthAcc){
	if(std::stoi(Utility::Instance().config["true_wind"])){
		float angle = heading+windDirection;
		angle = fmod(angle + M_PI, 2.0 * M_PI);
		float s = glm::length(v);
		float windAcc = sqrt(windAccx*windAccx+windAccy*windAccy);
		//float T = sqrt(s*s+windAcc*windAcc-(2*s*windAcc*cos(windDirection)));
		//float b = (windAcc*windAcc-T*T-s*s)/(2*T*s);
		//float the = acos(b);
		float dx = s*cos(heading)-windAcc*cos(angle);
		float dy = s*sin(heading)-windAcc*sin(angle);
		if(windNorthAcc != NULL)
			*windNorthAcc = sqrt(dx*dx+dy*dy);
		//return heading+the;
		return (dx==0&dy==0)?angle:atan2(dy,dx);
	}else{
		float angle = heading+windDirection;
		angle = fmod(angle + M_PI, 2.0 * M_PI);
    		if (angle < 0.0)
        		angle += 2.0 * M_PI;
    		return angle - M_PI;
	}
}


float Utility::TackingStrategy(float distanceToLine, float lineBearing, float windNorth, float heading, float corridor, float psi, float ksi, int *q){
	if(abs(distanceToLine) > corridor/2)
		(*q) = distanceToLine>=0?1:-1;

	if(cos(windNorth-heading)+cos(ksi) < 0 || (abs(distanceToLine) < corridor && (cos(windNorth-lineBearing)+cos(ksi) < 0)))
		heading = M_PI + windNorth - (*q)*ksi;
	return heading;
}

float Utility::TackingStrategy(float distanceToLine, float lineBearing, float windNorth, float heading, float corridor, float psi, float ksi, int *q, bool *check){
	if(abs(distanceToLine) > corridor/2)
		(*q) = distanceToLine>=0?1:-1;

	if(cos(windNorth-heading)+cos(ksi) < 0 || (abs(distanceToLine) < corridor && (cos(windNorth-lineBearing)+cos(ksi) < 0))){
		heading = M_PI + windNorth - (*q)*ksi;
		*check = true;
	}
	return heading;
}

glm::dvec2 Utility::StandardCommand(glm::dvec3 currentHeading, float heading, float windNorth, float max_sail, float max_rudder){
	glm::dvec2 rudsail;
	if(cos(currentHeading.z - heading) >= 0)
		rudsail.x = max_rudder*sin(currentHeading.z-heading);
	else
		rudsail.x = max_rudder*((sin(currentHeading.z-heading)>=0)?1:-1);

	rudsail.y = max_sail*(cos(windNorth-heading)+1)/2.0;
	return rudsail;
}

void Utility::simulateBoat(const double* p, double aaw, double daw, double atw, double dtw, double rudder, double sail, const double* state, double* outstate, double dt, bool control){
	double theta = state[2];
        double v = state[3];
        double w = state[4];
        if(control){
		double sigma = cos(daw)+cos(sail);
        	if (sigma<0){
                	sail = M_PI + daw;
        	}
        	else{
                	sail = -sign(sin(daw))*sail;
        	}
	}
        double fr = p[4]*v*v*sin(rudder);
        double fs = p[3]*aaw*sin(sail-daw);
        outstate[0] = state[0] + (v*cos(theta) + p[0]*atw*cos(dtw))*dt;
        outstate[1] = state[1] + (v*sin(theta) + p[0]*atw*sin(dtw))*dt;
        outstate[2] = theta + w*dt;
        outstate[3] = v + ((fs*sin(sail) - fr*sin(rudder)*p[10] - p[1]*v*v)/p[8])*dt;
        outstate[4] = state[4] + ((fs*(p[5]-p[6]*cos(sail))- p[7]*fr*cos(rudder)-p[2]*w*v)/p[9])*dt;
}
