#include "controller.hpp"

using namespace Sailboat;

Controller::Controller(std::string name, int looprate) : name(name), looprate(looprate){
    
}

void Controller::init(int argc, char **argv){
    ros::init(argc, argv, "controller");
    
    gpsSub = n.subscribe("gpu", 100, &Controller::gpsCallback, this);
    //imuSub = n.subscribe("imu", 100, chatterCallback);
    //windSub = n.subscribe("wind", 100, chatterCallback);
    
    pub = n.advertise<geometry_msgs::Vector3>("control", 100);
}


void Controller::control(){
    
}

void Controller::gps(const gps_common::GPSFix::ConstPtr& msg){
    
}

void Controller::imu(){
    
}

void Controller::wind(){
    
}
