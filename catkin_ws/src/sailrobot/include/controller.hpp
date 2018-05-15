#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "ros/ros.h"

#include <gps_common/GPSFix.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <gps_common/GPSFix.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <string>

namespace Sailboat{
    class Controller{
    public:
        enum MODE{STANDBY = 0, RUDDER_SAIL = 1, RETURN_HOME = 2, HEADING = 3, RC = 4, SAIL_CAP = 5, RUDDER = 6, WAYPOINTS = 7};
        
        Controller(std::string name, int looprate, int controller);
        ~Controller(){}
        
        void init(int argc, char **argv);
        void wakeup(const ros::TimerEvent& event);
        virtual void loop();
        
        void controlPublished();
        virtual void setup(ros::NodeHandle* n) = 0;
        virtual geometry_msgs::Twist control() = 0;
        
        virtual void gps(const gps_common::GPSFix::ConstPtr& msg);
        virtual void imu(const sensor_msgs::Imu::ConstPtr& msg);
        virtual void wind(const geometry_msgs::Pose2D::ConstPtr& msg);
        virtual void sail(const std_msgs::Float32::ConstPtr& msg);
        virtual void rudder(const std_msgs::Float32::ConstPtr& msg);
        virtual void rudder2(const std_msgs::Float32::ConstPtr& msg);
        virtual void vel(const geometry_msgs::Twist::ConstPtr& msg);
    protected:
        ros::NodeHandle* n;
        ros::Rate* loop_rate;
        
        ros::Timer timer;
        
        ros::Subscriber gpsSub;
        ros::Subscriber imuSub;
        ros::Subscriber windSub;
        ros::Subscriber sailSub;
        ros::Subscriber rudderSub;
        ros::Subscriber rudder2Sub;
        ros::Subscriber velSub;
        
        ros::Publisher odomMsg;
        ros::Publisher pubCmd;
        ros::Publisher pubMsg;
        
        std::string name;
        
        gps_common::GPSFix gpsMsg;
        sensor_msgs::Imu imuMsg;
        geometry_msgs::Pose2D windMsg;
        geometry_msgs::Twist velMsg;
		float sailAngle;
		float rudderAngle;
		float rudder2Angle;
        
        void publishCMD(geometry_msgs::Twist cmd);
        void publishMSG(std_msgs::String msg);
        void publishMSG(std::string msg);
        
        template <class T>
        T getParam(std::string name){T tmp; if(ros::param::get(name,tmp)) return tmp; return T();}
    private:
        void gpsCallback(const gps_common::GPSFix::ConstPtr& msg){gps(msg);}
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){imu(msg);}
        void windCallback(const geometry_msgs::Pose2D::ConstPtr& msg){wind(msg);}
        void sailCallback(const std_msgs::Float32::ConstPtr& msg){sail(msg);}
        void rudderCallback(const std_msgs::Float32::ConstPtr& msg){rudder(msg);}
        void rudder2Callback(const std_msgs::Float32::ConstPtr& msg){rudder(msg);}
        void velCallback(const geometry_msgs::Twist::ConstPtr& msg){vel(msg);}
		
	void publishOdom();
        
        int looprate;
        int controller;
    };
}

#endif

