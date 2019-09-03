#ifndef LINEFOLLOWING_HPP
#define LINEFOLLOWING_HPP

#include "controller.hpp"
#include <glm/glm.hpp>
#include <vector>

namespace Sailboat{
    class LineFollowing : public Controller{
	public:
        	LineFollowing(std::string name) : Controller(name,5, MODE::RUDDER_SAIL), r(10.0), currentWaypoint(0), nbWaypoints(0), q(1){}
        	~LineFollowing(){waypoints.clear();}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control();
	private:
		int currentWaypoint;
		int nbWaypoints;

		std::vector<glm::vec2> waypoints;
		float r;
		int q;
    };
}

#endif
