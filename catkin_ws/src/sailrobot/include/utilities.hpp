class Utility{
	public:
		Utility(){}
		static float GPSDist(float lat1, float lon1, float lat2, float lon2);
		static float GPSDistFast(float lat1, float lon1, float lat2, float lon2);
		static float GPSBearing(float lat1, float lon1, float lat2, float lon2);
		static float GPStoCartesian(float lat, float gpslong);
		static float CartesiantoGPS(float x, float y);
};
