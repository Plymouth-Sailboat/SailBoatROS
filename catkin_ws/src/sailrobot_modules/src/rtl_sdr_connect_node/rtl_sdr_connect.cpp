#include "rtl_sdr_connect_node/rtl_sdr_connect.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <ros/package.h>

int RTLSDRConnector::connect_socket(struct sockaddr_in* adr, int size,int sock) {
	std::cout << "CONNECTING TO SOCKET\n\n"<< std::endl;
	if(connect(sock,(struct sockaddr*) adr,size) ==-1)
	{
		perror("failed to connect");
		exit(1);
	}
	return sock;
}

int RTLSDRConnector::adr_init(struct sockaddr_in* adr){
	//initializes a socket 	
	(*adr).sin_port = htons(10110);
	(*adr).sin_family = AF_INET;
	return 0;
}

int RTLSDRConnector::receive_NMEA(int sock){
	// receives NMEA Strings from rtl_ais via TCP
	// the recv runs for 5 seconds 
	time_t start,end;
	double elapsed = 0;
	start = time(NULL);
	int b;
	std::cout << "5 SECONDS OF RECEPTION\n" << std::endl;
	while(elapsed <= 5){ // while 5 seconds 
		
		b = recv(sock,(void*) buffer,1000,MSG_DONTWAIT);
		if(b != 0 && b != -1){
	
			std::cout << buffer << std::endl;
			publish_buffer(buffer);}// calls		
	
		end = time(NULL);
		elapsed = difftime(end,start);;
	}
	std::cout << "END OF RECEPTION" << std::endl;
	shutdown(sock,SHUT_RDWR);
	close(sock);

	return 0;
}

int RTLSDRConnector::is_disconnected(int sock){
	char temp;
	//test whether the socket is connected or not 
	if(recv(sock,&temp,1,MSG_DONTWAIT)==0){
	
		std::cout << "SOCKET DECONNECTED" << std::endl;
		return 1;
	}
	else {
		return 0;
	}

}

int RTLSDRConnector::publish_buffer(char* buffer){
	msg.data = std::string(buffer);
	//NMEAPub.publish(msg);
	//std::cout << buffer << std::endl;

	//memset((void*) buffer,'\0',1000);//set buffer back to empty

	//sends strings to parse
	return 0;
}

void RTLSDRConnector::publish(){
	NMEAPub.publish(msg);
}

int main(int argc, char** argv){
	//init variables
	//RTLSDRConnector connector = RTLSDRConnector();
	struct sockaddr_in adr;
	
	struct sockaddr_in* pAdr = &adr;
		
	//connector.adr_init(&adr); 
	int size = sizeof(adr);

	std::string path = ros::package::getPath("sailrobot_modules") + "/bin/rtl-ais/rtl_ais -g 42 -p 0.6 -T &" ;
	system(path.c_str());
	sleep(5);

	ros::init(argc,argv, "rtl_sdr_connector");
	ros::NodeHandle n;
	RTLSDRConnector connector("NMEA", &n, &adr); 
	//connector.NMEAPub = n.advertise<std_msgs::String>("NMEA",1000);
	ros::Rate loop_rate(20); //TODO find the right frequency

	while(ros::ok()){
		int sock = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
		sock = connector.connect_socket(pAdr,size,sock);
		//std::cout << "after the connection"  << std::endl;

		std::thread rec(&RTLSDRConnector::receive_NMEA,&connector,sock);
		rec.join();
		
		connector.publish();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
