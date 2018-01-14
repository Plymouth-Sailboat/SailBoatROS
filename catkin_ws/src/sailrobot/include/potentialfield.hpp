#ifndef POTENTIALFIELD_HPP
#define POTENTIALFIELD_HPP

#include "controller.hpp"

namespace Sailboat{
    class PotentialField : public Controller{
	public:
        PotentialField(std::string name, int looprate) : Controller(name,looprate){}
        ~PotentialField(){}

		virtual void control();
	};
}

#endif
