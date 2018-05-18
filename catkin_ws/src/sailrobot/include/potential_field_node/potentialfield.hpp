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
		float** waypoints;
		float** obstacles;
		glm::vec3 distanceVector(glm::vec3 dest, glm::vec3 pos);
	};
}

#endif
