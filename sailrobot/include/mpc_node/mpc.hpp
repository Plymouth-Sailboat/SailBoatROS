#ifndef POTENTIALFIELD_HPP
#define POTENTIALFIELD_HPP

#include "controller.hpp"
#include <glm/glm.hpp>
#include <boost/bind.hpp>
#include <nlopt.hpp>

namespace Sailboat{
    class MPC : public Controller{
	public:
        MPC(std::string name) : Controller(name,2, MODE::RUDDER_SAIL), receding_n(10),step(1){}
        ~MPC(){}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control();

	private:
		int currentWaypoint;
		int nbWaypoints;
    nlopt::algorithm algo;
		std::vector<glm::dvec2> waypoints;
    double initXRef;
    double initYRef;
    std::vector<double> lb;
    std::vector<double> ub;

    int receding_n;
    unsigned int step;
    unsigned int inputs_n;

		static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *option);
		static double constraintFunction(unsigned n, const double *x, double *grad, void *data);
	};
}

#endif
