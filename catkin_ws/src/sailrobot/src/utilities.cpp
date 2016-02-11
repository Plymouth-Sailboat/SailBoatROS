#include <utilities.hpp>
#include <math.h>

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
