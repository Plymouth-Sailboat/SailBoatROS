#ifndef POTENTIALFIELD_HPP
#define POTENTIALFIELD_HPP

#include "controller.hpp"
#include <glm/glm.hpp>
#include <boost/bind.hpp>

namespace Sailboat{
    class MPC : public Controller{
	public:
        MPC(std::string name) : Controller(name,2, MODE::RUDDER_SAIL){}
        ~MPC(){}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control();

	private:
		int nbWaypoints;
		glm::vec2* waypoints;

		static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *option);
		static double constraintFunction(unsigned n, const double *x, double *grad, void *data);
	};
}

#endif
