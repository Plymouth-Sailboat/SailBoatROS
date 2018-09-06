#ifndef XBEE_END_HPP
#define XBEE_END_HPP

#include "controller.hpp"
#include <serial/serial.h>

namespace Xbee{
    class XbeeEndDevice : public Sailboat::Controller{
        public:
        	XbeeEndDevice(std::string name) : Sailboat::Controller(name,10, 0), serialXbee("/dev/ttyUSB0", 57600, serial::Timeout::simpleTimeout(1000)){}
        	~XbeeEndDevice(){}
                void setup(ros::NodeHandle* n);
                virtual geometry_msgs::Twist control(){return geometry_msgs::Twist();}
		bool loopUnpublished();
	
        private:
		serial::Serial serialXbee;
		void addBytes(float* data, int size, unsigned char id, int* pos, unsigned char* buffer);
        };
}
#endif
