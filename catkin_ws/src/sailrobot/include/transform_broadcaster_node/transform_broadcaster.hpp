#ifndef TRANSFORMBROADCASTER_HPP
#define TRANSFORMBROADCASTER_HPP

#include "controller.hpp"
#include <glm/glm.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace Sailboat{
    class TransformBroadcaster : public Controller{
	public:
        	TransformBroadcaster(std::string name) : Controller(name,10, MODE::HEADING){}
        	~TransformBroadcaster(){}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control(){}
		bool loopUnpublished();

	private:
		tf2_ros::TransformBroadcaster* broadcaster;
	};
}

#endif
