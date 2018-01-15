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
    
    pub = n->advertise<geometry_msgs::Vector3>("control", 100);
    pubMsg = n->advertise<std_msgs::String>("controller", 1);

    std::stringstream os;
    os << controller;
    std_msgs::String msg;
    msg.data = os.str();
    pubMsg.publish(msg);
}

void Controller::loop(){
    control();

    ros::spinOnce();
    loop_rate->sleep();
}


void Controller::control(){
    
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
