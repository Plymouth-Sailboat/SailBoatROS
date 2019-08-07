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
#include <gps_common/GPSFix.h>


using namespace std;
using namespace glm;

double xRef[2] = {0,0};

geometry_msgs::Pose2D wind;
geometry_msgs::Pose2D windN;
geometry_msgs::Twist cmd;

double delta_s;

//x,y,cap,v,w: vitesse rotation
double x[5] = {0,0,0,0,0};
glm::vec3 eulerA;

vec2 cubeA = {-10,0};
vec2 cubeB = {10,0};

double watchRc = 0;

string numberId;
string boatId, rudderId, sailId, windId, aId, bId, lineId;

/**********************************************************************/
void windNCB(const geometry_msgs::Pose2D::ConstPtr& msg){
	windN = *msg;
}
void windCB(const geometry_msgs::Pose2D::ConstPtr& msg){
	wind = *msg;
}

void rudder_sail_CB(const geometry_msgs::Twist::ConstPtr& msg){
	cmd = *msg;

	double theta = eulerA.z;
	double v = x[3];
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
}

void cubeACB(const geometry_msgs::Point msgA){

	cubeA[0] = 111.11*1000*(msgA.x-xRef[0]);
	cubeA[1] = -111.11*1000*(msgA.y-xRef[1])*cos(xRef[0]*M_PI/180);
}

void cubeBCB(const geometry_msgs::Point msgB){

	cubeB[0] = 111.11*1000*(msgB.x-xRef[0]);
	cubeB[1] = -111.11*1000*(msgB.y-xRef[1])*cos(xRef[0]*M_PI/180);
}

void refCB(const geometry_msgs::Point msgRef){
	xRef[0] = msgRef.x;
	xRef[1] = msgRef.y;
}

void eulerCB(const sensor_msgs::Imu msg){
	eulerA = glm::eulerAngles(glm::quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z));
}

void gpsCB(const gps_common::GPSFix msgGps){
	x[0] = 111.11*1000*(msgGps.latitude-xRef[0]);
	x[1] = -111.11*1000*(msgGps.longitude-xRef[1])*cos(xRef[0]*M_PI/180);
	x[2] = msgGps.track;
	x[3] = msgGps.speed;
}


void rcCB(const geometry_msgs::Vector3 msgRc){
	watchRc = msgRc.z;
	if (watchRc == 1){
		cmd.angular.x = msgRc.x;
		cmd.angular.y = msgRc.y;
		double theta = eulerA.z;
		double v = x[3];
		vec2 w_ap;
		float awind = sqrt(wind.x*wind.x+wind.y*wind.y);
		w_ap[0] = awind*cos(wind.theta-theta)-v;
		w_ap[1] = awind*sin(wind.theta-theta);
		double psi_ap = atan2(w_ap[1],w_ap[0]);
		double a_ap = glm::length(w_ap);
		double sigma = cos(psi_ap)+cos(cmd.angular.y);
		if (sigma<0){
			delta_s = M_PI + psi_ap;
		}
		else{
			delta_s = -sign(sin(psi_ap))*cmd.angular.y;
		}
	}
}
/***************************************************************************/

void set_marker_boat(ros::Publisher vis_pub, visualization_msgs::Marker marker){
	marker.header.frame_id = boatId;
	marker.header.stamp = ros::Time();
	marker.ns = boatId;
	marker.id = 0;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = -1.2;
	marker.pose.position.z = -1;
	tf::Quaternion q;
	q.setRPY(M_PI/2, 0, M_PI/2);
	tf::quaternionTFToMsg(q, marker.pose.orientation);
	marker.scale.x = 0.001;
	marker.scale.y = 0.001;
	marker.scale.z = 0.001;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://sailrobot/meshs/boat.STL";
	vis_pub.publish( marker );
}


void set_marker_rudder(visualization_msgs::Marker marker, ros::Publisher vis_pub)
{
	marker.header.frame_id = rudderId;
	marker.header.stamp = ros::Time();
	marker.ns = rudderId;
	marker.id = 0;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 1;
	marker.pose.position.y = 0.1;
	marker.pose.position.z = -2;
	tf::Quaternion q;
	q.setRPY(M_PI/2, 0, -M_PI/2);
	tf::quaternionTFToMsg(q, marker.pose.orientation);
	marker.scale.x = 0.003;
	marker.scale.y = 0.003;
	marker.scale.z = 0.003;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0;
	marker.color.b = 0;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://sailrobot/meshs/rudder.STL";
	vis_pub.publish( marker );
}


void set_marker_sail(visualization_msgs::Marker marker, ros::Publisher vis_pub)
{
	marker.header.frame_id = sailId;
	marker.header.stamp = ros::Time();
	marker.ns = sailId;
	marker.id = 0;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 2.7;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	tf::Quaternion q;
	q.setRPY(M_PI/2, 0, -M_PI/2);
	tf::quaternionTFToMsg(q, marker.pose.orientation);
	marker.scale.x = 0.001;
	marker.scale.y = 0.001;
	marker.scale.z = 0.001;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0;
	marker.color.g = 0;
	marker.color.b = 1.0;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://sailrobot/meshs/sail.STL";
	vis_pub.publish( marker );
}

void set_marker_wind(visualization_msgs::Marker marker, ros::Publisher vis_pub)
{
	marker.header.frame_id = windId;
	marker.header.stamp = ros::Time();
	marker.ns = windId;
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	tf::quaternionTFToMsg(q, marker.pose.orientation);
	marker.scale.x = 3.0;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0;
	marker.color.g = 1.0;
	marker.color.b = 0;
	//only if using a MESH_RESOURCE marker type:
	//marker.mesh_resource = "package://tp2/meshs/turret.dae";
	vis_pub.publish( marker );
}

void set_marker_A(visualization_msgs::Marker marker, ros::Publisher vis_pub)
{
	marker.header.frame_id = aId;
	marker.header.stamp = ros::Time();
	marker.ns = aId;
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	tf::quaternionTFToMsg(q, marker.pose.orientation);
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0;
	//only if using a MESH_RESOURCE marker type:
	//marker.mesh_resource = "package://tp2/meshs/turret.dae";
	vis_pub.publish( marker );
}

void set_marker_B(visualization_msgs::Marker marker, ros::Publisher vis_pub)
{
	marker.header.frame_id = bId;
	marker.header.stamp = ros::Time();
	marker.ns = bId;
	marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	tf::quaternionTFToMsg(q, marker.pose.orientation);
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0;
	marker.color.b = 1.0;
	//only if using a MESH_RESOURCE marker type:
	//marker.mesh_resource = "package://tp2/meshs/turret.dae";
	vis_pub.publish( marker );
}

void set_marker_line(visualization_msgs::Marker marker, ros::Publisher vis_pub){
	marker.header.frame_id = lineId;
	marker.header.stamp = ros::Time();
	marker.ns = lineId;
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	geometry_msgs::Point cA;
	geometry_msgs::Point cB;
	cA.x = cubeA[0];
	cA.y = cubeA[1];
	cA.z = 0;
	cB.x = cubeB[0];
	cB.y = cubeB[1];
	cB.z = 0;
	marker.points = {cA,cB};
	marker.scale.x = 0.5;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0;
	marker.color.g = 1;
	marker.color.b = 0;
	//only if using a MESH_RESOURCE marker type:
	//marker.mesh_resource = "package://tp2/meshs/turret.dae";
	vis_pub.publish( marker );
}


void initname(){
	boatId = "boat"+numberId;
	rudderId = "rudder"+numberId;
	sailId = "sail"+numberId;
	windId = "wind"+numberId;
	aId = "A"+numberId;
	bId = "B"+numberId;
	lineId = "line"+numberId;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "displayBoat");
	ros::NodeHandle nh;

	nh.param<string>("numberId",numberId,"1");
	nh.param<double>("posx", x[0],0);
	nh.param<double>("/simuBoat/xRefX", xRef[0],0);
	nh.param<double>("posy", x[1],0);
	nh.param<double>("/simuBoat/xRefY", xRef[1],0);
	nh.param<double>("theta", x[2],0);
	initname();

	cout << numberId << endl;

	ros::Subscriber sub_sail = nh.subscribe("/sailboat/sailboat_cmd",0,rudder_sail_CB);
	ros::Subscriber sub_wind = nh.subscribe("/sailboat/wind",0,windCB);
	ros::Subscriber sub_windN = nh.subscribe("/simu/windnorth",0,windNCB);
	ros::Subscriber sub_ref = nh.subscribe("control_send_ref",0,refCB);
	ros::Subscriber sub_euler = nh.subscribe("/sailboat/IMU",0,eulerCB);
	//ros::Subscriber sub_euler = nh.subscribe("ardu_send_euler_angles",0,eulerCB);
	ros::Subscriber sub_gps = nh.subscribe("/sailboat/GPS/fix",0,gpsCB);

	ros::Subscriber sub_A = nh.subscribe("control_send_A",0,cubeACB);
	ros::Subscriber sub_B = nh.subscribe("control_send_B",0,cubeBCB);

	ros::Subscriber sub_gps_xBee = nh.subscribe("xbee_send_gps_1",0,gpsCB);
	ros::Subscriber sub_wind_xBee = nh.subscribe("xbee_send_wind_direction_1",0,windCB);
	ros::Subscriber sub_euler_xbee = nh.subscribe("xbee_send_euler_1",0,eulerCB);
	ros::Subscriber sub_A_xbee = nh.subscribe("xbee_send_line_begin_1",0,cubeACB);
	ros::Subscriber sub_B_xbee = nh.subscribe("xbee_send_line_end_1",0,cubeBCB);



	ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0 );
	visualization_msgs::Marker marker;
	tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	transformStamped.child_frame_id = boatId;
	transformStamped.header.frame_id = "map";


	ros::Publisher vis_pub_rudder = nh.advertise<visualization_msgs::Marker>("visualization_marker_rudder", 0 );
	tf2_ros::TransformBroadcaster br_rudder;
	geometry_msgs::TransformStamped transformStamped_rudder;
	visualization_msgs::Marker marker_rudder;
	transformStamped_rudder.child_frame_id = rudderId;
	transformStamped_rudder.header.frame_id = boatId;

	ros::Publisher vis_pub_sail = nh.advertise<visualization_msgs::Marker>("visualization_marker_sail", 0 );
	tf2_ros::TransformBroadcaster br_sail;
	geometry_msgs::TransformStamped transformStamped_sail;
	visualization_msgs::Marker marker_sail;
	transformStamped_sail.child_frame_id = sailId;
	transformStamped_sail.header.frame_id = boatId;

	ros::Publisher vis_pub_wind = nh.advertise<visualization_msgs::Marker>("visualization_marker_wind", 0 );
	tf2_ros::TransformBroadcaster br_wind;
	geometry_msgs::TransformStamped transformStamped_wind;
	visualization_msgs::Marker marker_wind;
	transformStamped_wind.child_frame_id = windId;
	transformStamped_wind.header.frame_id = "map";


	ros::Publisher vis_pub_A = nh.advertise<visualization_msgs::Marker>("visualization_marker_A", 0 );
	tf2_ros::TransformBroadcaster br_A;
	geometry_msgs::TransformStamped transformStamped_A;
	visualization_msgs::Marker marker_A;
	transformStamped_A.child_frame_id = aId;
	transformStamped_A.header.frame_id = "map";

	ros::Publisher vis_pub_B = nh.advertise<visualization_msgs::Marker>("visualization_marker_B", 0 );
	tf2_ros::TransformBroadcaster br_B;
	geometry_msgs::TransformStamped transformStamped_B;
	visualization_msgs::Marker marker_B;
	transformStamped_B.child_frame_id = bId;
	transformStamped_B.header.frame_id = "map";


	ros::Publisher vis_pub_line = nh.advertise<visualization_msgs::Marker>("visualization_marker_line", 0 );
	visualization_msgs::Marker marker_line;
	tf2_ros::TransformBroadcaster br_line;
	geometry_msgs::TransformStamped transformStamped_line;
	transformStamped_line.child_frame_id = lineId;
	transformStamped_line.header.frame_id = "map";


	ros::Rate loop_rate(25);
	while (ros::ok()){
		ros::spinOnce();

		tf::Quaternion q;

		q.setRPY(eulerA.x,eulerA.y,eulerA.z);
		set_marker_boat( vis_pub,marker);
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.transform.translation.x = x[0];
		transformStamped.transform.translation.y = x[1];
		tf::quaternionTFToMsg(q, transformStamped.transform.rotation);
		br.sendTransform(transformStamped);

		q.setRPY(0, 0, cmd.angular.x+M_PI);
		set_marker_rudder(marker_rudder, vis_pub_rudder);
		transformStamped_rudder.header.stamp = ros::Time::now();
		transformStamped_rudder.transform.translation.x = 0;
		transformStamped_rudder.transform.translation.y = 0;
		tf::quaternionTFToMsg(q, transformStamped_rudder.transform.rotation);
		br_rudder.sendTransform(transformStamped_rudder);

		q.setRPY(0, 0, delta_s+M_PI);
		set_marker_sail(marker_sail, vis_pub_sail);
		transformStamped_sail.header.stamp = ros::Time::now();
		transformStamped_sail.transform.translation.x = 3;
		transformStamped_sail.transform.translation.y = 0;
		transformStamped_sail.transform.translation.z = 2;
		tf::quaternionTFToMsg(q, transformStamped_sail.transform.rotation);
		br_sail.sendTransform(transformStamped_sail);


		q.setRPY(0, 0, wind.theta);
		set_marker_wind(marker_wind, vis_pub_wind);
		transformStamped_wind.header.stamp = ros::Time::now();
		transformStamped_wind.transform.translation.x = x[0]+10;
		transformStamped_wind.transform.translation.y = x[1];
		transformStamped_wind.transform.translation.z = 2;
		tf::quaternionTFToMsg(q, transformStamped_wind.transform.rotation);
		br_wind.sendTransform(transformStamped_wind);



		q.setRPY(0, 0, 0);
		set_marker_A(marker_A, vis_pub_A);
		transformStamped_A.header.stamp = ros::Time::now();
		transformStamped_A.transform.translation.x = cubeA[0];
		transformStamped_A.transform.translation.y = cubeA[1];
		transformStamped_A.transform.translation.z = 0;
		tf::quaternionTFToMsg(q, transformStamped_A.transform.rotation);
		br_A.sendTransform(transformStamped_A);

		q.setRPY(0, 0, 0);
		set_marker_B(marker_B, vis_pub_B);
		transformStamped_B.header.stamp = ros::Time::now();
		transformStamped_B.transform.translation.x = cubeB[0];
		transformStamped_B.transform.translation.y = cubeB[1];
		transformStamped_B.transform.translation.z = 0;
		tf::quaternionTFToMsg(q, transformStamped_B.transform.rotation);
		br_B.sendTransform(transformStamped_B);


		transformStamped_line.header.stamp = ros::Time::now();
		transformStamped_line.transform.translation.x = 0;
		transformStamped_line.transform.translation.y = 0;
		transformStamped_line.transform.translation.z = 0;
		tf::quaternionTFToMsg(q, transformStamped_line.transform.rotation);
		br_line.sendTransform(transformStamped_line);
		set_marker_line(marker_line,vis_pub_line);


		loop_rate.sleep();
	}
	//sleep(1);
	ROS_INFO("FIN");
	return 0;
}

