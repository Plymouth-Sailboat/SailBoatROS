#include <iostream>
#include <vector>
#include <string>
#include "NMEA.h"
#include "check.h"
#include "ros/ros.h"
#include "std_msgs/String.h"




class Parser{



	public:
	char * champ_suivant(char* p);
	unsigned int nmea_uint(char* p);
	int end_trame(char* p,int i);
	char* create_vector(char* vector , char* p);
	void create_msg(char* buffer);
	int parser(char *pointeur);
	void chat_back(const std_msgs::String::ConstPtr& buffer);

//	private:
	
	ros::Publisher Data;
	ros::Subscriber trameNMEA;
	std_msgs::String msg;

};

