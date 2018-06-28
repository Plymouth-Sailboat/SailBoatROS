#include "controller.hpp"
#include <sstream>
#include <string>

using namespace Sailboat;

Controller::Controller(std::string name, int looprate, int controller) : name(name), looprate(looprate), controller(controller){

}

void Controller::init(int argc, char **argv){
	ros::init(argc, argv, name);

	n = new ros::NodeHandle("~");
	loop_rate = new ros::Rate(looprate);

	timer = n->createTimer(ros::Duration(30.0), &Controller::wakeup, this);

	gpsSub = n->subscribe("/sailboat/GPS", 100, &Controller::gpsCallback, this);
	imuSub = n->subscribe("/sailboat/IMU", 100, &Controller::imuCallback, this);
	windSub = n->subscribe("/sailboat/wind", 100, &Controller::windCallback, this);
	sailSub = n->subscribe("/sailboat/sail", 100, &Controller::sailCallback, this);
	rudderSub = n->subscribe("/sailboat/rudder", 100, &Controller::rudderCallback, this);
	rudder2Sub = n->subscribe("/sailboat/rudder2", 100, &Controller::rudder2Callback, this);
	velSub = n->subscribe("/sailboat/IMU_Dv", 100, &Controller::velCallback, this);

	odomMsg = n->advertise<nav_msgs::Odometry>("/sailboat/odom", 100);
	pubCmd = n->advertise<geometry_msgs::Twist>("/sailboat/sailboat_cmd", 100);
	pubMsg = n->advertise<std_msgs::String>("/sailboat/sailboat_msg", 10);

	setup(n);

	usleep(1000*1000);
	publishMSG("C" + std::to_string(controller));
}

void Controller::wakeup(const ros::TimerEvent& event){
	publishMSG("M");
}

void Controller::loop(){
	if(!loopUnpublished())
		controlPublished();
	publishOdom();

	ros::spinOnce();
	loop_rate->sleep();
}

void Controller::controlPublished(){
	publishCMD(control());
}


void Controller::publishOdom(){
	nav_msgs::Odometry odom_msg;
	odom_msg.pose.pose.position.x = gpsMsg.latitude;
	odom_msg.pose.pose.position.y = gpsMsg.longitude;
	odom_msg.pose.pose.position.z = gpsMsg.altitude;

	odom_msg.pose.pose.orientation = imuMsg.orientation;

	odom_msg.twist.twist = velMsg;

	odomMsg.publish(odom_msg);
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

void Controller::sail(const std_msgs::Float32::ConstPtr& msg){
	sailAngle = msg->data;
}

void Controller::rudder(const std_msgs::Float32::ConstPtr& msg){
	rudderAngle = msg->data;
}

void Controller::rudder2(const std_msgs::Float32::ConstPtr& msg){
	rudder2Angle = msg->data;
}

void Controller::vel(const geometry_msgs::Twist::ConstPtr& msg){
	velMsg = *msg;
}

