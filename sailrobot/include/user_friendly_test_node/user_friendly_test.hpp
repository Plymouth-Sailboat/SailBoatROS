#ifndef USERTEST_HPP
#define USERTEST_HPP

#include "controller.hpp"
#include <glm/glm.hpp>

namespace Sailboat{
	class UserTest : public Controller{
		public:
			UserTest(std::string name) : Controller(name,1, MODE::RUDDER_SAIL){}
			~UserTest(){}
			void setup(ros::NodeHandle* n);
			geometry_msgs::Twist control(){return geometry_msgs::Twist();}
			bool loopUnpublished();
		private:
	};
}

#endif
