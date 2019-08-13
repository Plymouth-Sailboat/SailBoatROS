#ifndef RELAY_HPP
#define RELAY_HPP

#include "controller.hpp"
#include <glm/glm.hpp>

namespace Sailboat{
    class Relay : public Controller{
	public:
        	Relay(std::string name) : Controller(name,10, MODE::RUDDER_SAIL){}
        	~Relay(){}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control();

		void twistMsg_callback(const geometry_msgs::Twist::ConstPtr& msg);
	private:
		geometry_msgs::Twist current_cmd;
		ros::Subscriber sub;
	};
}

#endif
