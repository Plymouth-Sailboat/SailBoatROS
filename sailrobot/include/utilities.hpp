/**
 * @file utilities.hpp
 * @brief Utility Class
 *
 * This class is used in conjunction with controllers. It is used for commonly used functions with sail robots.
 *
 * @author Ulysse Vautier
 * @version
 * @date 2018-09-05
 */
#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <geometry_msgs/Quaternion.h>
#include <string>
#include <map>
#include <vector>

class Utility{
	public:
		Utility(){}
		~Utility(){delete instance;}

		static Utility& Instance(){if(instance==nullptr) instance = new Utility(); return *instance;}
		/** Calculate the distance between two GPS coordinates
		 * @{
		 */
		static float GPSDist(float lat1, float lon1, float lat2, float lon2);
		static float GPSDist(glm::dvec2 point1, glm::dvec2 point2);
		static float GPSDistFast(float lat1, float lon1, float lat2, float lon2);
		static float GPSDistFast(glm::dvec2 point1, glm::dvec2 point2);
		/** @} */

		/** Calculate the initial bearing between two GPS coordinates
		 * @{
		 */
		static float GPSBearing(float lat1, float lon1, float lat2, float lon2);
		static float GPSBearing(glm::dvec2 point1, glm::dvec2 point2);
		/** @} */

		/** Transforms GPS coordinates to Cartesian coordinates
		 * @{
		 */
		static glm::dvec3 GPSToCartesian(float lat, float gpslong);
		/** @} */

		/** Transforms GPS coordinates to Cartesian coordinates
		 * @{
		 */
		static glm::dvec3 GPSToCartesian(glm::dvec2 gpsposition);
		/** @} */

		static float CartesianToGPS(float x, float y);

		/** Conversion between Quaternion to Euler and Euler to Quaternion
		 * @{
		 */
		static glm::dvec3 QuaternionToEuler(glm::quat q);
		static glm::dvec3 QuaternionToEuler(float x, float y, float z, float w);
		static glm::dvec3 QuaternionToEuler(geometry_msgs::Quaternion q);
		static glm::quat EulerToQuaternion(glm::dvec3 v);
		static glm::quat EulerToQuaternion(float x, float y, float z);
		/** @} */

		/// Read a GPS file. The GPS file must have each GPS coordinates (latitude, longitude) separated by an end line.
		static glm::dvec2* ReadGPSCoordinates(std::string filepath, int& size);
		static glm::dvec2* AppendGPSCoordinates(std::string filepath, int& size, glm::dvec2* list, int sizeList);
		static std::vector<glm::dvec2> AppendGPSCoordinates(std::string filepath, int& size, std::vector<glm::dvec2>* list);

		static std::map<std::string,std::string> ReadConfig(std::string filepath);
		static void SaveConfig(std::string filepath);

		static float RelativeToTrueWind(glm::dvec2 v, float heading, float windDirection, float windAccx, float windAccy, float* windNorthAcc = NULL);

		/** Tacking Strategy
		 * @{
		 */
		static float TackingStrategy(float distanceToLine, float lineBearing, float windNorth, float heading, float corridor, float psi, float ksi, int *q);
		static float TackingStrategy(float distanceToLine, float lineBearing, float windNorth, float heading, float corridor, float psi, float ksi, int *q, bool *check);
		/** @} */

		/** Standard rudder and sail control from Heading
		 * @{
		 */
		static glm::dvec2 StandardCommand(glm::dvec3 currentHeading, float heading, float windNorth, float max_sail = M_PI/2.0, float max_rudder = M_PI/4.0);

		static void simulateBoat(const double* p, double aaw, double daw, double atw, double dtw, double rudder, double sail, const double* state, double* outstate, double dt, bool control);
		/** @} */

		std::map<std::string,std::string> config;
	private:
		//Singleton
		static Utility* instance;
};
#endif
