#include <utilities.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>

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

float Utility::GPSDistFast(float lat1, float lon1, float lat2, float lon2){
	float ksi1 = lat1*M_PI/180.0;
        float ksi2 = lat2*M_PI/180.0;
	float lam1 = lon1*M_PI/180.0;
	float lam2 = lon2*M_PI/180.0;
	
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
	return atan2(y,x);
}

float Utility::GPStoCartesian(float lat, float gpslong){

}

float Utility::CartesiantoGPS(float x, float y){

}

geometry_msgs::Vector3 Utility::QuaternionToEuler(float x, float y, float z, float w){
	geometry_msgs::Vector3 euler;
	double sinr = +2.0 * (x * y + z * w);
	double cosr = +1.0 - 2.0 * (y * y + z * z);
	double roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (x * z - w * y);
	double pitch = 0;
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (x * w + y * z);
	double cosy = +1.0 - 2.0 * (z * z + w * w);  
	double yaw = atan2(siny, cosy);
	
	euler.x = pitch;
	euler.y = roll;
	euler.z = yaw;
	return euler;
}

geometry_msgs::Quaternion Utility::EulerToQuaternion(float x, float y, float z){
	geometry_msgs::Quaternion q;
	
	double cy = cos(z * 0.5);
	double sy = sin(z * 0.5);
	double cr = cos(y * 0.5);
	double sr = sin(y * 0.5);
	double cp = cos(x * 0.5);
	double sp = sin(x * 0.5);

	q.x = cy * cr * cp + sy * sr * sp;
	q.y = cy * sr * cp - sy * cr * sp;
	q.z = cy * cr * sp + sy * sr * cp;
	q.w = sy * cr * cp - cy * sr * sp;
	
	return q;
}

float** Utility::ReadGPSCoordinates(std::string filepath){
	float** coordinates = NULL;
	std::fstream file(filepath);
	if(!file){
		std::cerr << "GPS Reading : Couldn't open file" << std::endl;
		return coordinates;
	}
	std::string line;
	int nbLines = std::count(std::istreambuf_iterator<char>(file), 
             std::istreambuf_iterator<char>(), '\n');
	coordinates = new float*[nbLines];
	file.clear();
	file.seekg(0, std::ios::beg);
	int i = 0;
	while (std::getline(file, line)) {
		coordinates[i] = new float[2];
		std::string coords;
		std::stringstream stream(line);
		std::getline(stream, coords, ',');
		coordinates[i][0] = std::atof(coords.c_str());
		std::getline(stream, coords, ',');
		coordinates[i][1] = std::atof(coords.c_str());
		i++;
	}
	file.close();
	return coordinates;
}
