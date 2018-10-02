#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <strings.h>
#include <netinet/in.h>
#include <netdb.h>
#include <signal.h>
#include <string.h>
#include <thread>
#include <mutex>
#include <time.h>
#include <fcntl.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

class RTLSDRConnector{
	public:
		RTLSDRConnector(std::string name, ros::NodeHandle* n, struct sockaddr_in* adr){adr_init(adr); NMEAPub = n->advertise<std_msgs::String>(name,1000);}

		int connect_socket( struct sockaddr_in* adr,int size,int sock);
		int adr_init(struct sockaddr_in* adr);
		int receive_NMEA(int sock);
		int is_disconnected(int sock);
		int publish_buffer(char* buffer);
		void publish();
		int reception_loop();
		char buffer[1000];
		//int sock;

	private:
		std_msgs::String msg;
		ros::Publisher NMEAPub;
};
