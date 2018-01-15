#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "ros/ros.h"

#include <gps_common/GPSFix.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <string>

namespace Sailboat{
	class Controller{
	public:
        	Controller(std::string name, int looprate, int controller);
        	~Controller(){}
        
        	void init(int argc, char **argv);

		virtual void loop();

		void controlPublished();
		virtual void setup(ros::NodeHandle* n) = 0;
		virtual geometry_msgs::Twist control() = 0;
        
		template <class T>
		T getParam(std::string name){T tmp; if(ros::param::get(name,tmp)) return tmp; return T();}

        	virtual void gps(const sensor_msgs::NavSatFix::ConstPtr& msg);
        	virtual void imu(const sensor_msgs::Imu::ConstPtr& msg);
        	virtual void wind(const geometry_msgs::Pose2D::ConstPtr& msg);
	protected:
        	ros::NodeHandle* n;
        	ros::Rate* loop_rate;

        	ros::Subscriber gpsSub;
        	ros::Subscriber imuSub;
        	ros::Subscriber windSub;
        
        	ros::Publisher pubCmd;
                ros::Publisher pubMsg;
        
        	std::string name;

		sensor_msgs::NavSatFix* gpsMsg;
                sensor_msgs::Imu* imuMsg;
                geometry_msgs::Pose2D* windMsg;

		void publishCMD(geometry_msgs::Twist cmd);
                void publishMSG(std_msgs::String msg);
    	private:
        	void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){gps(msg);}
        	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){imu(msg);}
        	void windCallback(const geometry_msgs::Pose2D::ConstPtr& msg){wind(msg);}
        
        	int looprate;
		int controller;
	};
}

#endif
