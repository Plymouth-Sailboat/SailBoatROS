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
}

bool TransformBroadcaster::loopUnpublished(){
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "sailboat_hull";
	transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

	
        transformStamped.transform.rotation.x = imuMsg.orientation.w;
        transformStamped.transform.rotation.y = imuMsg.orientation.x;
        transformStamped.transform.rotation.z = imuMsg.orientation.y;
        transformStamped.transform.rotation.w = imuMsg.orientation.z;

	geometry_msgs::TransformStamped transformStampedR;
        transformStampedR.header.stamp = ros::Time::now();
        transformStampedR.header.frame_id = "sailboat_hull";
        transformStampedR.child_frame_id = "sailboat_rudder";
        transformStampedR.transform.translation.x = 0.0;
        transformStampedR.transform.translation.y = -0.2;
        transformStampedR.transform.translation.z = 0.0;

	quat rudderQuat = Utility::EulerToQuaternion(0.0,0.0,rudderAngle);
        transformStampedR.transform.rotation.x = rudderQuat.x;
        transformStampedR.transform.rotation.y = rudderQuat.y;
        transformStampedR.transform.rotation.z = rudderQuat.z;
        transformStampedR.transform.rotation.w = rudderQuat.w;




        geometry_msgs::TransformStamped transformStampedS;
        transformStampedS.header.stamp = ros::Time::now();
        transformStampedS.header.frame_id = "sailboat_hull";
        transformStampedS.child_frame_id = "sailboat_sail";
        transformStampedS.transform.translation.x = 0.0;
        transformStampedS.transform.translation.y = 0.05;
        transformStampedS.transform.translation.z = 0.0;

        quat sailQuat = Utility::EulerToQuaternion(0.0,0.0,sailAngle);
        transformStampedS.transform.rotation.x = sailQuat.x;
        transformStampedS.transform.rotation.y = sailQuat.y;
        transformStampedS.transform.rotation.z = sailQuat.z;
        transformStampedS.transform.rotation.w = sailQuat.w;


	broadcaster->sendTransform(transformStamped);
        broadcaster->sendTransform(transformStampedR);
        broadcaster->sendTransform(transformStampedS);

	return true;
}

