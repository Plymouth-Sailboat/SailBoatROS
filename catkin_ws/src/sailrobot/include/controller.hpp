#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "ros/ros.h"

namespace Sailboat{
	class Controller{
	public:
		Controller(){}
		~Controller(){}

		virtual void control();
	private:
	};
}

#endif
