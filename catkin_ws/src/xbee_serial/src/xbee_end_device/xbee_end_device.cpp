#include "xbee_end_device/xbee_end_device.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <serial/serial.h>

using namespace Xbee;

void XbeeEndDevice::setup(ros::NodeHandle* n){}

void XbeeEndDevice::addBytes(float* data, int size, unsigned char id, int* pos, unsigned char* buffer){
	buffer[(*pos)++] = id;
	buffer[(*pos)++] = size;
        memcpy(buffer+(*pos),data,size);
        *pos += size;
}

bool XbeeEndDevice::loopUnpublished(){
	int pos = 0;
	unsigned char buffer[256];

	buffer[pos++] = 255;
	buffer[pos++] = 255;


        float quaternion[4] = {(float)imuMsg.orientation.x, (float)imuMsg.orientation.y, (float)imuMsg.orientation.z, (float)imuMsg.orientation.w};

	addBytes(quaternion, sizeof(quaternion), 0, &pos, buffer);

	float gps[2] = {(float)gpsMsg.latitude, (float)gpsMsg.longitude};

	addBytes(gps, sizeof(gps), 1, &pos, buffer);

	buffer[pos++] = 125;
	buffer[pos++] = 125;

	if(serialXbee.isOpen()){
		serialXbee.write(buffer, pos);
		
	}else
		std::cerr << "not connected to XBee" << std::endl;

	return true;
}
