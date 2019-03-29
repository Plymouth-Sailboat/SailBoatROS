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
		virtual unsigned char* build(T msg, size_t* pos = NULL){return NULL;}
		virtual T getData(){return parsed;}
	protected:
		void addBytes(float* data, int size, unsigned char id, size_t* pos, unsigned char* buffer);
		T parsed;
};

class XImu : public XParser<sensor_msgs::Imu>{
	public:
		int parse(unsigned char* data);
		unsigned char* build(sensor_msgs::Imu data, size_t* pos = NULL);
};

class XGps : public XParser<gps_common::GPSFix>{
        public:
                int parse(unsigned char* data);
                unsigned char* build(gps_common::GPSFix msg, size_t* pos = NULL);
};

class XPose2D : public XParser<geometry_msgs::Pose2D>{
        public:
                int parse(unsigned char* data);
                unsigned char* build(geometry_msgs::Pose2D msg, size_t* pos = NULL);
};

class XString : public XParser<std_msgs::String>{
        public:
                int parse(unsigned char* data);
                unsigned char* build(std_msgs::String msg, size_t* pos = NULL);
};

class XFloat32 : public XParser<std_msgs::Float32>{
        public:
                int parse(unsigned char* data);
                unsigned char* build(std_msgs::Float32 msg, size_t* pos = NULL);
};

class XTwist : public XParser<geometry_msgs::Twist>{
        public:
                int parse(unsigned char* data);
                unsigned char* build(geometry_msgs::Twist msg, size_t* pos = NULL);
};

class XbeeParser{
	public:
		XbeeParser();
		~XbeeParser();
		void parse(unsigned char* data, size_t size);

		template<class T>
		T getData(unsigned char id){return (T)(((XParser<T>*)parsers[id])->getData());}
		XParserInterface* getParser(unsigned char id){return parsers[id];}
	private:
		std::map<unsigned char, XParserInterface*> parsers;	
};
}
#endif
