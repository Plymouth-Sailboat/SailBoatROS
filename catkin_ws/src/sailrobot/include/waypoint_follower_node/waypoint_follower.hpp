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
		glm::vec2* waypoints;
		glm::vec2* tackingStart;
		float rmax;
		float closeHauled;
		int q;
	};
}

#endif