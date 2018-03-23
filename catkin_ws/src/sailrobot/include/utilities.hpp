class Utility{
	public:
		Utility(){}
		static float GPSDist(float lat1, float long1, float lat2, float long2);
		static float GPSAngle(float lat1, float long1, float lat2, float long2);
		static float GPStoCartesian(float lat, float gpslong);
		static float CartesiantoGPS(float x, float y);
		static float RudderByCap(float theta);
};
