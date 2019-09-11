#ifndef IDENTIFICATION_HPP
#define IDENTIFICATION_HPP

#include "controller.hpp"
#include <glm/glm.hpp>
#include <ctime>
#include <iostream>
#include <fstream>
#include <vector>
#include <array>

namespace Sailboat{
	class Identification : public Controller{
		public:
			Identification(std::string name) : Controller(name,10, MODE::RUDDER_SAIL), step(0), doSimu(0) {}
			~Identification(){if(data.is_open())data.close();}
			void setup(ros::NodeHandle* n);
			virtual geometry_msgs::Twist control();

		private:
			unsigned int step;
			double initXRef;
			double initYRef;
			glm::vec2 initPos;
			glm::vec2 goal1;
			float initWindA;
			float initWind;
			float initTheta;
			float initV;
			double start;

			std::vector<double> vnorm_list;

			int doSimu;

			std::ofstream data;
			std::ofstream dataState;
			std::vector<std::array<double, 10>> state;
			static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *option);
	};
}

#endif
