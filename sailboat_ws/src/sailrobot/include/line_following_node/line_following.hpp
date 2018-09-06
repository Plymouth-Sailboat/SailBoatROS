#ifndef LINEFOLLOWING_HPP
#define LINEFOLLOWING_HPP

#include "controller.hpp"
#include <glm/glm.hpp>

namespace Sailboat{
    class LineFollowing : public Controller{
	public:
        LineFollowing(std::string name) : Controller(name,10, MODE::RUDDER){}
        ~LineFollowing(){}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control();

	private:
		glm::vec2* waypoints;
	};
}

#endif
