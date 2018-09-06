#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "gps_common/GPSFix.h"
#include "serial/serial.h"
#include <sstream>

#include "xbee_parser.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "xbee_publisher");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<sensor_msgs::Imu>("/sailboat/IMU2", 1000);
	ros::Publisher pub2 = n.advertise<gps_common::GPSFix>("/sailboat/GPS2", 1000);
	ros::Rate loop_rate(10);

	serial::Serial serialXbee("/dev/ttyUSB1", 57600, serial::Timeout::simpleTimeout(1000));
	if(!serialXbee.isOpen())
		std::cerr << "Xbee not connect" << std::endl;

	Xbee_Parser::XbeeParser parser;
	while (ros::ok())
	{
		unsigned char buffer[256];
		if(int si = serialXbee.available()){
			serialXbee.read(buffer,si);	

			int pos = 0;
			bool found = false;
			for(int i = 0; i < si; ++i){
				if(buffer[i] == 255 && buffer[i+1] == 255){
					pos = i+2;
					found = true;
					break;
				}	
			}

			if(!found)
				continue;

			std::cout << "received : " << si << " data : " << (int)buffer[pos] << std::endl;
                        for(int i = pos; i < si; ++i)
                                std::cout << (int)buffer[i] << " ";
                        std::cout << std::endl;

			if(buffer[pos++] <= si-pos+2)
				continue;

			parser.parse(buffer+pos, buffer[pos]-2);
			pub.publish(parser.getData<sensor_msgs::Imu>(0));
			pub2.publish(parser.getData<gps_common::GPSFix>(1));
		}

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
