#include <utilities.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iterator>
#include <algorithm>

using namespace glm;

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

float Utility::GPStoCartesian(float lat, float gpslong){
    
}

float Utility::CartesiantoGPS(float x, float y){
    
}

vec3 Utility::QuaternionToEuler(float x, float y, float z, float w){
    return eulerAngles(quat(x,y,z,w));
}

quat Utility::EulerToQuaternion(float x, float y, float z){
    return quat(vec3(x,y,z));
}

vec2* Utility::ReadGPSCoordinates(std::string filepath){
    vec2* coordinates = NULL;
    std::fstream file(filepath);
    if(!file){
        std::cerr << "GPS Reading : Couldn't open file" << std::endl;
        return coordinates;
    }
    std::string line;
    int nbLines = std::count(std::istreambuf_iterator<char>(file),
                             std::istreambuf_iterator<char>(), '\n');
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

