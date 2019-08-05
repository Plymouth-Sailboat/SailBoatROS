#include "relay_node/relay.hpp"
#include <math.h>
#include <fstream>
#include <iostream>
#include <utilities.hpp>
#include <ros/package.h>

using namespace Sailboat;
using namespace glm;

void Relay::twistMsg_callback(const geometry_msgs::Twist::ConstPtr& msg){
	current_cmd = *msg;
}

void Relay::setup(ros::NodeHandle* n){
	std::string topic = "xbee_send_rudder_sail";
	if (n->hasParam("topic"))
		n->getParam("topic",topic);

	sub = n->subscribe(topic, 100, &Relay::twistMsg_callback, this);
}

geometry_msgs::Twist Relay::control(){
	publishLOG("Sending : " + std::to_string(current_cmd.angular.x) + ", " + std::to_string(current_cmd.angular.y));
	return current_cmd;
}

