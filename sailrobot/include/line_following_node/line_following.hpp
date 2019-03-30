#ifndef LINEFOLLOWING_HPP
#define LINEFOLLOWING_HPP

#include "controller.hpp"
#include <glm/glm.hpp>

namespace Sailboat{
    class LineFollowing : public Controller{
	public:
        	LineFollowing(std::string name) : Controller(name,10, MODE::RUDDER), r(10.0), currentWaypoint(0), nbWaypoints(0){}
        	~LineFollowing(){}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control();
	private:
		int currentWaypoint;
		int nbWaypoints;

		glm::vec2* waypoints;
		float r;
    };
}

#endif
