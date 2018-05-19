#ifndef AREASCANNING_HPP
#define AREASCANNING_HPP

#include "controller.hpp"
#include <glm/glm.hpp>

namespace Sailboat{
    class AreaScanning : public Controller{
	public:
        AreaScanning(std::string name) : Controller(name,10, MODE::HEADING){}
        ~AreaScanning(){}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control();

	private:
		int currentWaypoint;
		int nbWaypoints;
		glm::vec2* waypoints;
		int* waypointsOrder;

		void buildLKHFiles();
		void readResults();
	};
}

#endif
