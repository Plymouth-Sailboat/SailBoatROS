/**
 * @file waypoint_follower.hpp
 * @class WaypointFollower
 * @brief Waypoint Follower Class
 *
 * This is the implementation for a waypoint follower for an autonomous sailboat
 *
 * @author Ulysse Vautier
 * @version
 * @date 2018-09-05
 */
#ifndef WAYPOINTFOLLOWER_HPP
#define WAYPOINTFOLLOWER_HPP

#include "controller.hpp"
#include <glm/glm.hpp>

namespace Sailboat{
    class WaypointFollower : public Controller{
	public:
        WaypointFollower(std::string name) : Controller(name,10, MODE::HEADING), tackingStart(NULL), rmax(20), q(1){}
        ~WaypointFollower(){}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control();

	private:
		int currentWaypoint;
		int nbWaypoints;
		glm::dvec2* waypoints;
		glm::dvec2* tackingStart;
		float rmax;
		float closeHauled;
		int q;
	};
}

#endif
