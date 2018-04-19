#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

class Utility{
	public:
		Utility(){}
		static float GPSDist(float lat1, float lon1, float lat2, float lon2);
		static float GPSDistFast(float lat1, float lon1, float lat2, float lon2);
		static float GPSBearing(float lat1, float lon1, float lat2, float lon2);
		static float GPStoCartesian(float lat, float gpslong);
		static float CartesiantoGPS(float x, float y);
		static geometry_msgs::Vector3 QuaternionToEuler(x,y,z,w);
		static geometry_msgs::Quaternion EulerToQuaternion(x, y, z);
};
