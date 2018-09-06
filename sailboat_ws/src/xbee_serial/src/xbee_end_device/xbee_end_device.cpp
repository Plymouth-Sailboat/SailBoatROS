#include "xbee_end_device/xbee_end_device.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <serial/serial.h>

#include "xbee_parser.h"

using namespace Xbee;

void XbeeEndDevice::setup(ros::NodeHandle* n){}

void XbeeEndDevice::addBytes(float* data, int size, unsigned char id, int* pos, unsigned char* buffer){
	buffer[(*pos)++] = size;
	buffer[(*pos)++] = id;
        memcpy(buffer+(*pos),data,size);
        *pos += size;
}

bool XbeeEndDevice::loopUnpublished(){
	int pos = 0;
	unsigned char buffer[256];

	buffer[pos++] = 255;
	buffer[pos++] = 255;
	buffer[pos++] = 0;

	Xbee_Parser::XbeeParser parser;

	size_t size = 0;
	unsigned char* quaternion = ((Xbee_Parser::XImu*)parser.getParser(0))->build(imuMsg,&size);
	memcpy(buffer+pos, quaternion, size);
	pos+= size;

	size = 0;
	unsigned char* gps = ((Xbee_Parser::XGps*)parser.getParser(1))->build(gpsMsg,&size);
	memcpy(buffer+pos,gps,size);
	pos += size;

	buffer[pos++] = 125;
	buffer[pos++] = 125;

	buffer[2] = pos;

	delete quaternion;
	delete gps;

	if(serialXbee.isOpen()){
		serialXbee.write(buffer, pos);
		
	}else
		std::cerr << "not connected to XBee" << std::endl;

	return true;
}
