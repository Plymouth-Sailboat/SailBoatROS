#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "ros/ros.h"

#include <gps_common/GPSFix.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>

namespace Sailboat{
	class Controller{
	public:
        	Controller(std::string name, int looprate);
        	~Controller(){}
        
        	void init(int argc, char **argv);

		virtual void loop();

		virtual void control() = 0;
        
        	virtual void gps(const sensor_msgs::NavSatFix::ConstPtr& msg);
        	virtual void imu(const sensor_msgs::Imu::ConstPtr& msg);
        	virtual void wind(const geometry_msgs::Pose2D::ConstPtr& msg);
	protected:
        	ros::NodeHandle* n;
        	ros::Rate* loop_rate;

        	ros::Subscriber gpsSub;
        	ros::Subscriber imuSub;
        	ros::Subscriber windSub;
        
        	ros::Publisher pub;
        
        	std::string name;

		sensor_msgs::NavSatFix* gpsMsg;
                sensor_msgs::Imu* imuMsg;
                geometry_msgs::Pose2D* windMsg;

    	private:
        	void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){gps(msg);}
        	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){imu(msg);}
        	void windCallback(const geometry_msgs::Pose2D::ConstPtr& msg){wind(msg);}
        
        	int looprate;
	};
}

#endif
