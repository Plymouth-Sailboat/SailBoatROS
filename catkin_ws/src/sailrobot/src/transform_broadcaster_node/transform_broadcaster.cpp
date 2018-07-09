#include "transform_broadcaster_node/transform_broadcaster.hpp"
#include <math.h>
#include <fstream>
#include <iostream>
#include <utilities.hpp>
#include <ros/package.h>

using namespace Sailboat;
using namespace glm;

void TransformBroadcaster::setup(ros::NodeHandle* n){
	broadcaster = new tf2_ros::TransformBroadcaster();
	ros::NodeHandle nh;
	jointPub = nh.advertise<sensor_msgs::JointState>("joint_states",100);

        joint.name.push_back("rudder_angle");
        joint.name.push_back("sail_angle");
        joint.position.push_back(rudderAngle);
        joint.position.push_back(sailAngle);

}

bool TransformBroadcaster::loopUnpublished(){
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "base_hull";
	transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

	
        transformStamped.transform.rotation.x = imuMsg.orientation.w;
        transformStamped.transform.rotation.y = imuMsg.orientation.x;
        transformStamped.transform.rotation.z = imuMsg.orientation.y;
        //transformStamped.transform.rotation.w = imuMsg.orientation.z;

        transformStamped.transform.rotation.w = 1;

	joint.header.stamp = ros::Time::now();
	joint.position[0] = rudderAngle;
	joint.position[1] = sailAngle;

	broadcaster->sendTransform(transformStamped);
	jointPub.publish(joint);
	return true;
}

