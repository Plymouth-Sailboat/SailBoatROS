#include "gpsfix_tcp/TCPServer.h"
//#include "gpsfix_tcp/gpsfix_tcp.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

#define PORT 10110

TCPServer tcp;
std_msgs::String msg;

void tcp_send(const std_msgs::String::ConstPtr& buffer){
	char* copymsg = new char[buffer->data.length()];
	memcpy(copymsg,buffer->data.c_str(),buffer->data.length());
	msg = *buffer;
	std::cout << "Sending : " << buffer->data << std::endl;
	tcp.Send(buffer->data.c_str());
	tcp.clean();
	delete copymsg;

}

int main(int argc,char** argv){ 	
	//string trame;
	pthread_t msgsender;
	tcp.setup(PORT);

	ros::init(argc,argv,"gpsfix_tcp");
	ros::NodeHandle n;

	ros::Subscriber trameNMEA = n.subscribe("sailboat/GPS/NMEA",1000,&tcp_send);

	ros::Rate loop_rate(20);
	while(ros::ok()){
		socklen_t sosize  = sizeof(tcp.clientAddress);
		tcp.newsockfd = accept(tcp.sockfd,(struct sockaddr*)&tcp.clientAddress,&sosize);
		ros::spinOnce();
		loop_rate.sleep();
	}
	tcp.detach();
	return 0;
}

