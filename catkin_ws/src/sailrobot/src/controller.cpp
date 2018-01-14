#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include "controller.hpp"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  ros::NodeHandle n;

  ros::Publisher controller_pub = n.advertise<std_msgs::String>("controller_pub", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    controller_pub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
// %EndTag(FULLTEXT)%
