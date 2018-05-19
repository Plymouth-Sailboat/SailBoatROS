#ifndef LINEFOLLOWING_HPP
#define LINEFOLLOWING_HPP

#include "controller.hpp"
#include <glm/glm.hpp>

namespace Sailboat{
    class WaypointFollower : public Controller{
	public:
        WaypointFollower(std::string name) : Controller(name,10, MODE::HEADING){}
        ~WaypointFollower(){}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control();

	private:
		int currentWaypoint;
		int nbWaypoints;
		glm::vec2* waypoints;
	};
}

#endif
