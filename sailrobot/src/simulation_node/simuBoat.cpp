#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <visualization_msgs/Marker.h>
#include "tf/tf.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <gps_common/GPSFix.h>


using namespace std;
using namespace glm;

double xRef[2] = {0,0};

geometry_msgs::Pose2D wind;
geometry_msgs::Pose2D windN;
geometry_msgs::Twist cmd;

double delta_s;
double p[10] = {0.1,1,6000,1000,2000,1,1,2,300,10000};

//x,y,cap,v,w: vitesse rotation
double x[5] = {0,0,0,0,0};
double xdot[5] = {0,0,0,0,0};


double t0;

vec2 cubeA = {-10,0};
vec2 cubeB = {10,0};

double buoy[2] = {50.691,-4.235};


void windCB(const geometry_msgs::Pose2D::ConstPtr& msg){
	windN = *msg;
}

void rudder_sail_CB(const geometry_msgs::Twist::ConstPtr& msg){
	cmd = *msg;
}

void refCB(const geometry_msgs::Point msgRef){
	xRef[0] = msgRef.x;
	xRef[1] = msgRef.y;
}

/***********************************************************************************************/

void f(){
	double theta = x[2];
	double v = x[3];
	double w = x[4];
	vec2 w_ap;
	float awind = sqrt(windN.x*windN.x+windN.y*windN.y);
	w_ap[0] = awind*cos(windN.theta-theta)-v;
	w_ap[1] = awind*sin(windN.theta-theta);
	double psi_ap = atan2(w_ap[1],w_ap[0]);
	double a_ap = glm::length(w_ap);
	double sigma = cos(psi_ap)+cos(cmd.angular.y);
	if (sigma<0){
		delta_s = M_PI + psi_ap;
	}
	else{
		delta_s = -sign(sin(psi_ap))*cmd.angular.y;
	}
	double fr = p[4]*v*sin(cmd.angular.x);
	double fs = p[3]*a_ap*sin(delta_s-psi_ap);
	xdot[0] = v*cos(theta) + p[0]*awind*cos(wind.theta);
	xdot[1] = v*sin(theta) + p[0]*awind*sin(wind.theta);
	xdot[2] = w;
	xdot[3] = (fs*sin(delta_s) - fr*sin(cmd.angular.x) - p[1]*v*v)/p[8];
	xdot[4] = (fs*(p[5]-p[6]*cos(delta_s))- p[7]*fr*cos(cmd.angular.x)-p[2]*w*v)/p[9];

}


void act(){
	double t1 = ros::Time::now().toSec();
	double dt = t1 - t0;
	//double dt = 0.1;
	x[0] = x[0] + dt*xdot[0];
	x[1] = x[1] + dt*xdot[1];
	x[2] = x[2] + dt*xdot[2];
	x[3] = x[3] + dt*xdot[3];
	x[4] = x[4] + dt*xdot[4];
	t0 = t1;
}

/********************************************************************************************************************/

void set_imu(ros::Publisher pub_imu, sensor_msgs::Imu msgImu, double x[5]){
	glm::quat quat(glm::vec3(0, 0, x[2])); 

	msgImu.orientation.x=quat.x;
	msgImu.orientation.y=quat.y;
	msgImu.orientation.z=quat.z;
	msgImu.orientation.w=quat.w;

	msgImu.linear_acceleration.x = 0;
	msgImu.linear_acceleration.y = 0;
	msgImu.linear_acceleration.z = 0;

	msgImu.angular_velocity.x  = 0;
	msgImu.angular_velocity.y  = 0;
	msgImu.angular_velocity.z  = 0;
	pub_imu.publish(msgImu);
}

void set_gps(ros::Publisher pub_gps, gps_common::GPSFix msgGps, double x[5]){
	msgGps.latitude = x[0]/(111.11*1000)+ xRef[0];
	msgGps.longitude = -x[1]/(111.11*1000*cos(xRef[0]*M_PI/180))+xRef[1];
	msgGps.track = x[2];
	msgGps.speed = x[3];
	pub_gps.publish(msgGps);
}


void set_wind(ros::Publisher pub_wind, geometry_msgs::Pose2D msgWind){
	double v = x[3];
        vec2 w_ap;
        float awind = sqrt(windN.x*windN.x+windN.y*windN.y);
        w_ap[0] = awind*cos(windN.theta-x[2])-v;
        w_ap[1] = awind*sin(windN.theta-x[2]);
	double psi_ap = atan2(w_ap[1],w_ap[0]);
        double a_ap = glm::length(w_ap);

	msgWind.theta = psi_ap;
	msgWind.x = w_ap[0];
	msgWind.y = w_ap[1];
	pub_wind.publish(msgWind);
}

void set_buoy(ros::Publisher pub_buoy, geometry_msgs::Vector3 msgBuoy){
	double dx = 111.11*1000*(buoy[0]-x[0]);
	double dy = -111.11*1000*(buoy[1]-x[1])*cos(x[0]*M_PI/180);
	double distance = sqrt(dx*dx+dy*dy);
	double angle  = acos(dx/(distance*1));
	if (dy < 0){
		angle = -angle;
	}
	msgBuoy.x = distance;
	msgBuoy.y = angle-x[2];
	pub_buoy.publish(msgBuoy);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "simuBoat");
	ros::NodeHandle nh;

	ros::Subscriber sub_rudder = nh.subscribe("/sailboat/sailboat_cmd",0,rudder_sail_CB);
	ros::Subscriber sub_ref = nh.subscribe("control_send_ref",0,refCB);

	ros::Publisher  pub_imu = nh.advertise<sensor_msgs::Imu>("/sailboat/IMU",0);
	sensor_msgs::Imu msgImu;
	ros::Publisher  pub_imu_Dv = nh.advertise<geometry_msgs::Twist>("/sailboat/IMU_Dv",0);
	geometry_msgs::Twist msgImu_Dv;
	ros::Publisher pubGps = nh.advertise<gps_common::GPSFix>("/sailboat/GPS/fix",0);
	gps_common::GPSFix msgGps;
	ros::Publisher pub_wind = nh.advertise<geometry_msgs::Pose2D>("/sailboat/wind",0);
	geometry_msgs::Pose2D msgWind;
	ros::Publisher pub_windN = nh.advertise<geometry_msgs::Pose2D>("/simu/windnorth",0);

	ros::Publisher pub_buoy = nh.advertise<geometry_msgs::Vector3>("simu_send_buoy",0);
	geometry_msgs::Vector3 msgBuoy;



	t0 = ros::Time::now().toSec();
	nh.param<double>("posx", x[0],0);
	nh.param<double>("/simuBoat/xRefX", xRef[0],0);
	nh.param<double>("posy", x[1],0);
	nh.param<double>("/simuBoat/xRefY", xRef[1],0);
	nh.param<double>("theta", x[2],0);
	x[3] = 1;
	x[4] = 0;
	wind = geometry_msgs::Pose2D();
	windN = geometry_msgs::Pose2D();
	windN.x = 2;
	cmd = geometry_msgs::Twist();
	msgImu_Dv = geometry_msgs::Twist();

	ros::Rate loop_rate(25);
	while (ros::ok()){
		ros::spinOnce();
		f();
		act();
		set_gps(pubGps,msgGps,x);
		set_imu(pub_imu, msgImu,x);
		set_wind(pub_wind,msgWind);
		set_buoy(pub_buoy,msgBuoy);
		pub_windN.publish(windN);

		msgImu_Dv.linear.x = cos(x[2])*x[3];
		msgImu_Dv.linear.y = cos(x[3])*x[3];
		pub_imu_Dv.publish(msgImu_Dv);

		loop_rate.sleep();
	}
	//sleep(1);
	ROS_INFO("FIN");
	return 0;
}
