#ifndef IDENTIFICATION_HPP
#define IDENTIFICATION_HPP

#include "controller.hpp"
#include <glm/glm.hpp>
#include <ctime>

namespace Sailboat{
    class Identification : public Controller{
	public:
        Identification(std::string name) : Controller(name,10, MODE::RUDDER_SAIL), step(0){}
        ~Identification(){}
	void setup(ros::NodeHandle* n);
	virtual geometry_msgs::Twist control();

	private:
		unsigned int step;
		glm::vec2 initPos;
		glm::vec2 goal1;
		float initWind;
		std::clock_t start;
	};
}

#endif
