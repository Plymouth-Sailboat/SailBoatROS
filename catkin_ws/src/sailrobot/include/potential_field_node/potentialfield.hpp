#ifndef POTENTIALFIELD_HPP
#define POTENTIALFIELD_HPP

#include "controller.hpp"

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <vector>

namespace Sailboat{
    class PotentialField : public Controller{
	public:
        PotentialField(std::string name) : Controller(name,10, MODE::HEADING){}
        ~PotentialField(){}
		void setup(ros::NodeHandle* n);
		virtual geometry_msgs::Twist control();

		void waypoint_callback(const geometry_msgs::Point::ConstPtr& waypoints);
                void obstacles_callback(const geometry_msgs::Point::ConstPtr& obspoints);

	private:
		std::vector<geometry_msgs::Point> wpoints;
		std::vector<geometry_msgs::Point> opoints;

		ros::Subscriber subWays;
		ros::Subscriber subObs;

		tf::Vector3 toXYZ(tf::Vector3 pos);
		tf::Vector3 distanceVector(tf::Vector3 dest, tf::Vector3 pos);
	};
}

#endif
