#include "controller.hpp"
#include <utilities.hpp>
#include <sstream>
#include <string>

using namespace Sailboat;

Controller::Controller(std::string name, int looprate, int controller) : name(name), looprate(looprate), controller(controller), rudderAngle(0), sailAngle(0), xbeeMode(0), isSetup(false), beginLoop(false){

}

void Controller::init(int argc, char **argv){
	ros::init(argc, argv, name);

	n = new ros::NodeHandle("~");
	loop_rate = new ros::Rate(looprate);


	gpsSub = n->subscribe("/sailboat/GPS/fix", 100, &Controller::gpsCallback, this);
	imuSub = n->subscribe("/sailboat/IMU", 100, &Controller::imuCallback, this);
	windSub = n->subscribe("/sailboat/wind", 100, &Controller::windCallback, this);
	sailSub = n->subscribe("/sailboat/sail", 100, &Controller::sailCallback, this);
	rudderSub = n->subscribe("/sailboat/rudder", 100, &Controller::rudderCallback, this);
	rudder2Sub = n->subscribe("/sailboat/rudder2", 100, &Controller::rudder2Callback, this);
	velSub = n->subscribe("/sailboat/IMU_Dv", 100, &Controller::velCallback, this);
	xbeeSub = n->subscribe("xbee_send_mode", 100, &Controller::xbeeCallback, this);
	xbeeCSub = n->subscribe("xbee_send_rudder_sail", 100, &Controller::xbeeRudSailCallback, this);

	odomMsg = n->advertise<nav_msgs::Odometry>("/sailboat/odom", 100);
	pubCmd = n->advertise<geometry_msgs::Twist>("/sailboat/sailboat_cmd", 100);
	pubMsg = n->advertise<std_msgs::String>("/sailboat/sailboat_msg", 10);
	pubLog = n->advertise<std_msgs::String>("/sailboat/pc_log", 10);
	pubMarkerA = n->advertise<geometry_msgs::Point>("/control_send_A", 10);
	pubMarkerB = n->advertise<geometry_msgs::Point>("/control_send_B", 10);

	std::string configPath = "config/config.txt";
	if(n->hasParam("config"))
		n->getParam("config",configPath);
	Utility::Instance().config = Utility::ReadConfig(configPath);

	ros::Duration(0.1).sleep();

	if(!loopUnpublished()){
		timer = n->createTimer(ros::Duration(30.0), &Controller::wakeup, this);
		publishMSG("C" + std::to_string(controller));
	}
}

void Controller::wakeup(const ros::TimerEvent& event){
	publishMSG("C" + std::to_string(controller));
}

void Controller::loop(){
	if(isSetup && !beginLoop){
		setup(n);
		beginLoop = true;
	}

	if(beginLoop){
		if(!loopUnpublished())
			controlPublished();
		publishOdom();

	}
	ros::spinOnce();
	loop_rate->sleep();
}

void Controller::controlPublished(){
	if(xbeeMode != 1)
		publishCMD(control());
	else
		publishCMD(xbeeControl);
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

void Controller::publishLOG(std_msgs::String msg){
	pubLog.publish(msg);
}

void Controller::publishLOG(std::string msg){
	std_msgs::String msgD;
	msgD.data = msg.c_str();
	pubLog.publish(msgD);
}

void Controller::publishMarkerA(float latitude, float longitude){
	geometry_msgs::Point point;
	point.x = latitude;
	point.y = longitude;
	pubMarkerA.publish(point);
}

void Controller::publishMarkerB(float latitude, float longitude){
	geometry_msgs::Point point;
	point.x = latitude;
	point.y = longitude;
	pubMarkerB.publish(point);
}

void Controller::gps(const gps_common::GPSFix::ConstPtr& msg){
	gpsMsg = *msg;
	isSetup = true;
}

void Controller::imu(const sensor_msgs::Imu::ConstPtr& msg){
	imuMsg = *msg;
}

void Controller::wind(const geometry_msgs::Pose2D::ConstPtr& msg){
	windMsg = *msg;
	/*windMsg.x = msg->x;
	  windMsg.y = msg->y;
	  windAvg.push_back(msg->theta);
	  if(windAvg.size() > 50)
	  windAvg.erase(windAvg.begin());
	  float ws = 0;
	  float wc = 0;
	  float w = 0;
	  for(int i = 0; i < windAvg.size(); ++i){
	  ws += sin(windAvg[i]);
	  wc += cos(windAvg[i]);
	  }
	  w = atan2(ws,wc);
	  windMsg.theta = w;*/
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

void Controller::mode(const std_msgs::UInt32::ConstPtr& msg){
	modeControl = msg->data;
}

void Controller::vel(const geometry_msgs::Twist::ConstPtr& msg){
	velMsg = *msg;
}

void Controller::xbeeCallback(const std_msgs::Float32::ConstPtr& msg){
	xbeeMode = msg->data;
}

void Controller::xbeeRudSailCallback(const geometry_msgs::Twist::ConstPtr& msg){
	xbeeControl = *msg;
}
