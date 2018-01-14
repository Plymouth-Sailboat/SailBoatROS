#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "ros/ros.h"

#include <gps_common/GPSFix.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <string>

namespace Sailboat{
	class Controller{
	public:
        Controller(std::string name, int looprate);
        ~Controller();
        
        void init(int argc, char **argv);

		virtual void control() = 0;
        
        virtual void gps(const gps_common::GPSFix::ConstPtr& msg);
        virtual void imu(const sensor_msgs::Imu::ConstPtr& msg);
        virtual void wind(const geometry_msgs::Vector3::ConstPtr& msg);
	protected:
        ros::NodeHandle n;
        
        ros::Subscriber gpsSub;
        ros::Subscriber imuSub;
        ros::Subscriber windSub;
        
        ros::Publisher pub;
        
        std::string name;
    private:
        void gpsCallback(const gps_common::GPSFix::ConstPtr& msg){gps(msg);}
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){imu(msg);}
        void windCallback(const geometry_msgs::Vector3::ConstPtr& msg){wind(msg);}
        
        int looprate;
	};
}

#endif
