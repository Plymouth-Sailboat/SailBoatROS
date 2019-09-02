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

float Utility::GPSDist(vec2 point1, vec2 point2){
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

float Utility::GPSDistFast(vec2 point1, vec2 point2){
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

float Utility::GPSBearing(vec2 point1, vec2 point2){
	float ksi1 = point1.x*M_PI/180.0;
	float ksi2 = point2.x*M_PI/180.0;
	float lam1 = point1.y*M_PI/180.0;
	float lam2 = point2.y*M_PI/180.0;

	float y = sin(lam2-lam1)*cos(ksi2);
	float x = cos(ksi1)*sin(ksi2)-sin(ksi1)*cos(ksi2)*cos(lam2-lam1);
	return -atan2(y,x);
}

glm::vec3 Utility::GPSToCartesian(float lat, float gpslong){
	float ksi1 = lat*M_PI/180.0;
	float lam1 = gpslong*M_PI/180.0;
	return glm::vec3(6371000*cos(ksi1)*cos(lam1),
			6371000*cos(ksi1)*sin(lam1),
			6371000*sin(ksi1));
}

glm::vec3 Utility::GPSToCartesian(glm::vec2 gpsposition){
	return GPSToCartesian((float)gpsposition.x,(float)gpsposition.y);
}

float Utility::CartesianToGPS(float x, float y){

}

vec3 Utility::QuaternionToEuler(geometry_msgs::Quaternion q){
	return eulerAngles(quat(q.w, q.x, q.y, q.z));
}

vec3 Utility::QuaternionToEuler(quat q){
	return eulerAngles(q);
}

vec3 Utility::QuaternionToEuler(float x, float y, float z, float w){
	return eulerAngles(quat(w,x,y,z));
}

quat Utility::EulerToQuaternion(vec3 v){
	return quat(v);
}

quat Utility::EulerToQuaternion(float x, float y, float z){
	return quat(vec3(x,y,z));
}

vec2* Utility::ReadGPSCoordinates(std::string filepath, int& size){
	vec2* coordinates = NULL;
	std::string path = ros::package::getPath("sailrobot");
	if(filepath[0] != '/')
		filepath = path + "/" + filepath;
	std::fstream file(filepath);
	if(!file){
		std::cerr << "GPS Reading : Couldn't open file" << std::endl;
		return coordinates;
	}
	std::string line;
	int nbLines = std::count(std::istreambuf_iterator<char>(file),
			std::istreambuf_iterator<char>(), '\n');
	size = nbLines;
	coordinates = new vec2[nbLines];
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


vec2* Utility::AppendGPSCoordinates(std::string filepath, int& size, glm::vec2* list, int sizeList){
	vec2* coordinates = NULL;
	std::string path = ros::package::getPath("sailrobot");
	if(filepath[0] != '/')
		filepath = path + "/" + filepath;
	std::fstream file(filepath);
	if(!file){
		std::cerr << "GPS Reading : Couldn't open file" << std::endl;
		return coordinates;
	}
	std::string line;
	int nbLines = std::count(std::istreambuf_iterator<char>(file),
			std::istreambuf_iterator<char>(), '\n');
	size = nbLines;
	coordinates = new vec2[nbLines+sizeList];
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

std::vector<vec2> Utility::AppendGPSCoordinates(std::string filepath, int& size, std::vector<glm::vec2>* list){
	std::string path = ros::package::getPath("sailrobot");
	if(filepath[0] != '/')
		filepath = path + "/" + filepath;
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
		list->push_back(glm::vec2(x,y));
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

float Utility::RelativeToTrueWind(glm::vec2 v, float heading, float windDirection, float windAccx, float windAccy){
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

glm::vec2 Utility::StandardCommand(glm::vec3 currentHeading, float heading, float windNorth, float max_sail, float max_rudder){
	glm::vec2 rudsail;
	if(cos(currentHeading.z - heading) >= 0)
		rudsail.x = max_rudder*sin(currentHeading.z-heading);
	else
		rudsail.x = max_rudder*((sin(currentHeading.z-heading)>=0)?1:-1);

	rudsail.y = max_sail*(cos(windNorth-heading)+1)/2.0;
	return rudsail;
}
