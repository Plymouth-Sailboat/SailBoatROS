#include "potentialfield.hpp"
#include <std_msgs/String.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard : [%s]", msg->data.c_str());
}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    Sailboat::PotentialField controller("Controller", 10, 0);
    controller.init(argc, argv);
    
//	ros::init(argc, argv, "Controller");
//	ros::NodeHandle n;
//	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
//    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
	controller.loop();
        //controller_pub.publish(msg);
//        ros::spinOnce();
        
//        loop_rate.sleep();
    }
    
    
    return 0;
}
// %EndTag(FULLTEXT)%

