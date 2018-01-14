#include "controller.hpp"

using namespace Sailboat;

Controller::Controller(std::string name, int looprate) : name(name), looprate(looprate){
    
}

void Controller::init(int argc, char **argv){
    ros::init(argc, argv, name);
    
    gpsSub = n.subscribe("gpu", 100, &Controller::gpsCallback, this);
    imuSub = n.subscribe("imu", 100, &Controller::imuCallback, this);
    windSub = n.subscribe("wind", 100, &Controller::windCallback, this);
    
    pub = n.advertise<geometry_msgs::Vector3>("control", 100);
}


void Controller::control(){
    
}

void Controller::gps(const gps_common::GPSFix::ConstPtr& msg){
    
}

void Controller::imu(const sensor_msgs::Imu::ConstPtr& msg){
    
}

void Controller::wind(const geometry_msgs::Vector3::ConstPtr& msg){
    
}
