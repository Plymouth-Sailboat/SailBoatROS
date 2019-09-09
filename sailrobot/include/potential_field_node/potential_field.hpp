#ifndef POTENTIALFIELD_HPP
#define POTENTIALFIELD_HPP

#include "controller.hpp"
#include <glm/glm.hpp>

namespace Sailboat{
    class PotentialField : public Controller{
	public:
        PotentialField(std::string name) : Controller(name,2, MODE::HEADING){}
        ~PotentialField(){}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control();

	private:
		int nbWaypoints;
		int nbObstacles;
		glm::dvec2* waypoints;
		glm::dvec2* obstacles;
		glm::dvec2 distanceVector(glm::dvec2 dest, glm::dvec2 pos);

		float closeHauled;
	};
}

#endif
