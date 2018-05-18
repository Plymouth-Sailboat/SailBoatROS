#ifndef POTENTIALFIELD_HPP
#define POTENTIALFIELD_HPP

#include "controller.hpp"
#include <glm/glm.hpp>

namespace Sailboat{
    class PotentialField : public Controller{
	public:
        PotentialField(std::string name) : Controller(name,10, MODE::HEADING){}
        ~PotentialField(){}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control();

	private:
		glm::vec2* waypoints;
		glm::vec2* obstacles;
		glm::vec2 distanceVector(glm::vec2 dest, glm::vec2 pos);
	};
}

#endif
