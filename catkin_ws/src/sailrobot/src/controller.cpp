#include "controller.hpp"
#include <sstream>


using namespace Sailboat;

Controller::Controller(std::string name, int looprate, int controller) : name(name), looprate(looprate), controller(controller){
    
}

void Controller::init(int argc, char **argv){
    ros::init(argc, argv, name);

    n = new ros::NodeHandle();   
    loop_rate = new ros::Rate(looprate);
 
    gpsSub = n->subscribe("GPS", 100, &Controller::gpsCallback, this);
    imuSub = n->subscribe("IMU", 100, &Controller::imuCallback, this);
    windSub = n->subscribe("Wind", 100, &Controller::windCallback, this);
    
    pubCmd = n->advertise<geometry_msgs::Twist>("sailboat_cmd", 100);
    pubMsg = n->advertise<std_msgs::String>("sailboat_msg", 10);
}

void Controller::loop(){
    controlPublished();

    ros::spinOnce();
    loop_rate->sleep();
}

void Controller::controlPublished(){
	publishCMD(control());
}

void Controller::publishCMD(geometry_msgs::Twist msg){
	pubCmd.publish(msg);
}

void Controller::publishMSG(std_msgs::String msg){
        pubMsg.publish(msg);
}


void Controller::gps(const sensor_msgs::NavSatFix::ConstPtr& msg){
    *gpsMsg = *msg;
}

void Controller::imu(const sensor_msgs::Imu::ConstPtr& msg){
    *imuMsg = *msg;
}

void Controller::wind(const geometry_msgs::Pose2D::ConstPtr& msg){
    *windMsg = *msg;
}
