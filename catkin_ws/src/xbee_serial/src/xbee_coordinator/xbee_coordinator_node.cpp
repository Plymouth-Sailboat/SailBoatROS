#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "gps_common/GPSFix.h"
#include "serial/serial.h"
#include <sstream>



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
	while (ros::ok())
	{
		unsigned char buffer[256];
		if(int si = serialXbee.available()){
			serialXbee.read(buffer,si);	

			int pos = 0;
			for(int i = 0; i < 255; ++i){
				if(buffer[i] == 255 && buffer[i+1] == 255){
					pos = i+2;
					break;
				}	
			}
			std::cout << "received : " << si << std::endl;
                        for(int i = pos; i < si; ++i)
                                std::cout << (int)buffer[i] << " ";
                        std::cout << std::endl;

			if(pos + sizeof(float)*6 < 256){
				float orientation[4];
				memcpy(orientation, buffer+pos+2, sizeof(float)*4);
				sensor_msgs::Imu imu;
				imu.orientation.x = orientation[0];
				imu.orientation.y = orientation[1];
				imu.orientation.z = orientation[2];
				imu.orientation.w = orientation[3];
				pub.publish(imu);

				float gps[2];
				memcpy(gps,buffer+pos+4+sizeof(float)*4, sizeof(float)*2);

				gps_common::GPSFix gpsM;
				gpsM.latitude = gps[0];
				gpsM.longitude = gps[1];

				pub2.publish(gpsM);

				serialXbee.flush();
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
