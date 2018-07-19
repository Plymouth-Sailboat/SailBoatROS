#ifndef XBEE_PARSER_HPP
#define XBEE_PARSER_HPP

#include <map>
#include "gps_common/GPSFix.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

namespace Xbee_Parser{

class XParserInterface{
	public:
		virtual int parse(unsigned char*) = 0;
};

template<typename T> class XParser : public XParserInterface{
	public:
		XParser(){}
		virtual int parse(unsigned char*){return 0;}
		virtual T getData(){return parsed;}
	protected:
		T parsed;
};

class XImu : public XParser<sensor_msgs::Imu>{
	public:
		int parse(unsigned char* data);
};

class XGps : public XParser<gps_common::GPSFix>{
        public:
                int parse(unsigned char* data);
};

class XPose2D : public XParser<geometry_msgs::Pose2D>{
        public:
                int parse(unsigned char* data);
};

class XString : public XParser<std_msgs::String>{
        public:
                int parse(unsigned char* data);
};

class XFloat32 : public XParser<std_msgs::Float32>{
        public:
                int parse(unsigned char* data);
};

class XTwist : public XParser<geometry_msgs::Twist>{
        public:
                int parse(unsigned char* data);
};

class XbeeParser{
	public:
		XbeeParser();
		~XbeeParser();
		void parse(unsigned char* data, size_t size);

		template<class T>
		T getData(unsigned char id){return (T)(((XParser<T>*)parsers[id])->getData());}
	private:
		std::map<unsigned char, XParserInterface*> parsers;	
};
}
#endif
