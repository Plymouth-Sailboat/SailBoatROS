#include "controller.hpp"
#include <sstream>
#include <string>

using namespace Sailboat;

Controller::Controller(std::string name, int looprate, int controller) : name(name), looprate(looprate), controller(controller){
    
}

void Controller::init(int argc, char **argv){
    ros::init(argc, argv, name);

    n = new ros::NodeHandle();   
    loop_rate = new ros::Rate(looprate);
 
    timer = n->createTimer(ros::Duration(30.0), &Controller::wakeup, this);

    gpsSub = n->subscribe("/sailboat/GPS", 100, &Controller::gpsCallback, this);
    imuSub = n->subscribe("/sailboat/IMU", 100, &Controller::imuCallback, this);
    windSub = n->subscribe("/sailboat/wind", 100, &Controller::windCallback, this);
    
    pubCmd = n->advertise<geometry_msgs::Twist>("/sailboat/sailboat_cmd", 100);
    pubMsg = n->advertise<std_msgs::String>("/sailboat/sailboat_msg", 10);

    setup(n);

    usleep(1000*1000);

    std_msgs::String msg;
    msg.data = "C" + std::to_string(controller);
    pubMsg.publish(msg);
}

void Controller::wakeup(const ros::TimerEvent& event){
    publishMSG("M");
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

void Controller::publishMSG(std::string msg){
	std_msgs::String msgD;
	msgD.data = msg.c_str();
        pubMsg.publish(msgD);
}


void Controller::gps(const gps_common::GPSFix::ConstPtr& msg){
	gpsMsg = *msg;
}

void Controller::imu(const sensor_msgs::Imu::ConstPtr& msg){
	imuMsg = *msg;
}

void Controller::wind(const geometry_msgs::Pose2D::ConstPtr& msg){
	windMsg = *msg;
}
