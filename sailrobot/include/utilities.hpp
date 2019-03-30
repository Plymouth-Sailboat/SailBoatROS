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

class Utility{
	public:
		Utility(){}
		/** Calculate the distance between two GPS coordinates
		 * @{
		 */
		static float GPSDist(float lat1, float lon1, float lat2, float lon2);
		static float GPSDist(glm::vec2 point1, glm::vec2 point2);
		static float GPSDistFast(float lat1, float lon1, float lat2, float lon2);
		static float GPSDistFast(glm::vec2 point1, glm::vec2 point2);		
		/** @} */

		/** Calculate the initial bearing between two GPS coordinates
		 * @{
		 */
		static float GPSBearing(float lat1, float lon1, float lat2, float lon2);
		static float GPSBearing(glm::vec2 point1, glm::vec2 point2);
		/** @} */

		static float GPStoCartesian(float lat, float gpslong);
		static float CartesiantoGPS(float x, float y);

		/** Conversion between Quaternion to Euler and Euler to Quaternion
		 * @{
		 */
		static glm::vec3 QuaternionToEuler(glm::quat q);
		static glm::vec3 QuaternionToEuler(float x, float y, float z, float w);
		static glm::vec3 QuaternionToEuler(geometry_msgs::Quaternion q);
		static glm::quat EulerToQuaternion(glm::vec3 v);
		static glm::quat EulerToQuaternion(float x, float y, float z);
		/** @} */

		/// Read a GPS file. The GPS file must have each GPS coordinates (latitude, longitude) separated by an end line.
		static glm::vec2* ReadGPSCoordinates(std::string filepath, int& size);


		/** Tacking Strategy
		 * @{
		 */
		static float TackingStrategy(float distanceToLine, float lineBearing, float windNorth, float heading, float corridor, float psi, float ksi);
		/** @} */

		/** Standard rudder and sail control from Heading
		 * @{
		 */
		static glm::vec2 StandardCommand(glm::vec3 currentHeading, float heading, float windNorth, float max_sail = M_PI/2.0, float max_rudder = M_PI/4.0);
		/** @} */
};
#endif
