#include "potentialfield.hpp"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    Sailboat::PotentialField controller("Controller", 10);
    controller.init(argc, argv);
    
    ros::Rate loop_rate(10);
    
    while (ros::ok())
    {
        //controller_pub.publish(msg);
        ros::spinOnce();
        
        loop_rate.sleep();
    }
    
    
    return 0;
}
// %EndTag(FULLTEXT)%

