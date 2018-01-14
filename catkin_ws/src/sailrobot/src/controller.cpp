#include "controller.hpp"

using namespace Sailboat;

Controller::Controller(std::string name, int looprate) : name(name), looprate(looprate){
    
}

void Controller::init(int argc, char **argv){
    ros::init(argc, argv, "controller");
    
    gpu = n.subscribe("gpu", 100, &Controller::gpsCallback, this);
    //imu = n.subscribe("imu", 100, chatterCallback);
    //wind = n.subscribe("wind", 100, chatterCallback);
    
    pub = n.advertise("control", 100);
}


virtual void Controller::control(){
    
}

virtual void Controller::gps(const gps_common::GPSFix::ConstPtr& msg){
    
}

virtual void Controller::imu(){
    
}

virtual void Controller::wind(){
    
}
