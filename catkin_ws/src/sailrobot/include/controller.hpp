#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "ros/ros.h"

#include <gps_common/GPSFix.h>
#include <geometry_msgs/Vector3.h>
#include <string>

namespace Sailboat{
	class Controller{
	public:
        Controller(std::string name, int looprate);
        ~Controller();
        
        void init(int argc, char **argv);

		virtual void control();
        
        virtual void gps(const gps_common::GPSFix::ConstPtr& msg);
        virtual void imu();
        virtual void wind();
	protected:
        ros::NodeHandle n;
        
        ros::Subscriber gpsSub;
        ros::Subscriber imuSub;
        ros::Subscriber windSub;
        
        ros::Publisher pub;
        
        std::string name;
    private:
        void gpsCallback(const gps_common::GPSFix::ConstPtr& msg){gps(msg);}
        void imuCallback(){imu();}
        void windCallback(){wind();}
        
        int looprate;
	};
}

#endif
