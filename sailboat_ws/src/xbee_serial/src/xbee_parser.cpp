#include "xbee_parser.h"

using namespace Xbee_Parser;

XbeeParser::~XbeeParser(){
	  for (std::map<unsigned char, XParserInterface*>::iterator it=parsers.begin(); it!=parsers.end(); ++it)
		delete it->second;
}

XbeeParser::XbeeParser(){
	parsers[0] = new XImu();
	parsers[1] = new XGps();
	parsers[2] = new XPose2D();
	parsers[3] = new XString();
	parsers[4] = new XFloat32();
	parsers[5] = new XTwist();
}

void XbeeParser::parse(unsigned char* data, size_t size){
	int pos = 0;
	while(pos < size-2){
		pos += parsers[data[pos+1]]->parse(data+pos);
	}
}

int XImu::parse(unsigned char* data){
	int size = data[0]+2;
	float orientation[10];
	memcpy(orientation,data+2,size-2);
	parsed.orientation.x = orientation[0];
        parsed.orientation.y = orientation[1];
        parsed.orientation.z = orientation[2];
        parsed.orientation.w = orientation[3];

	parsed.angular_velocity.x = orientation[4];
	parsed.angular_velocity.y = orientation[5];
	parsed.angular_velocity.z = orientation[6];

	parsed.linear_acceleration.x = orientation[7];
	parsed.linear_acceleration.y = orientation[8];
	parsed.linear_acceleration.z = orientation[9];
	return size;
}

unsigned char* XImu::build(sensor_msgs::Imu msg, size_t* pos){
	unsigned char* data = new unsigned char[sizeof(float)*10+2];
	float orientation[10] = {(float)msg.orientation.x, (float)msg.orientation.y, (float)msg.orientation.z, (float)msg.orientation.w, (float)msg.angular_velocity.x, (float)msg.angular_velocity.y, (float)msg.angular_velocity.z, (float)msg.linear_acceleration.x, (float)msg.linear_acceleration.y, (float)msg.linear_acceleration.z};
	addBytes(orientation, sizeof(float)*10, 0, pos, data);
	return data;
}

int XGps::parse(unsigned char* data){
        int size = data[0]+2;
        float gps[2];
        memcpy(gps,data+2,size-2);
        parsed.latitude = gps[0];
        parsed.longitude = gps[1];
	parsed.altitude = gps[2];
	parsed.track = gps[3];
	parsed.speed = gps[4];
	parsed.time = gps[5];
	parsed.hdop = gps[6];
        return size;
}

unsigned char* XGps::build(gps_common::GPSFix msg, size_t* pos){
	unsigned char* data = new unsigned char[sizeof(float)*7+2];
	float gps[7] = {(float)msg.latitude, (float)msg.longitude, (float)msg.altitude, (float)msg.track, (float)msg.speed, (float)msg.time, (float)msg.hdop};
	addBytes(gps, sizeof(float)*7, 1, pos, data);
	return data;
}

int XPose2D::parse(unsigned char* data){
        int size = data[0]+2;
        float orientation[4];
        memcpy(orientation,data+2,size-2);
        return size;
}

unsigned char* XPose2D::build(geometry_msgs::Pose2D msg, size_t* pos){
	unsigned char* data = new unsigned char[2];
        if(pos != NULL)
                *pos += 2;

	return data;
}

int XString::parse(unsigned char* data){
        int size = data[0]+2;
        std::string msg(data+2,data+2+size);
	parsed.data = msg;
        return size;
}

unsigned char* XString::build(std_msgs::String msg, size_t* pos){
	unsigned char* data = new unsigned char[2];
	data[0] = msg.data.length();
	data[1] = 2;
	std::string dataS(msg.data);
	memcpy(data+2, dataS.c_str(), dataS.length());

        if(pos != NULL)
                *pos += dataS.length()+2;

	return data;
}

int XFloat32::parse(unsigned char* data){
        int size = data[0]+2;
        float value;
        memcpy(&value,data+2,size-2);
	parsed.data = value;
        return size;
}

unsigned char* XFloat32::build(std_msgs::Float32 msg, size_t* pos){
	unsigned char* data = new unsigned char[sizeof(float)];
	data[0] = sizeof(float);
	data[1] = 4;
	float value = msg.data;
	memcpy(data, &value, sizeof(float));

        if(pos != NULL)
                *pos += sizeof(float)+2;

	return data;
}

int XTwist::parse(unsigned char* data){
        int size = data[0]+2;
        float linearandangular[6];
        memcpy(linearandangular,data+2,size-2);
        parsed.linear.x = linearandangular[0];
        parsed.linear.y = linearandangular[1];
        parsed.linear.z = linearandangular[2];
        parsed.angular.x = linearandangular[3];
        parsed.angular.y = linearandangular[4];
        parsed.angular.z = linearandangular[5];

        return size;
}

unsigned char* XTwist::build(geometry_msgs::Twist msg, size_t* pos){
	unsigned char* data = new unsigned char[2];

        if(pos != NULL)
                *pos += 2;

	return data;
}

template<class T>
void XParser<T>::addBytes(float* data, int size, unsigned char id, size_t* pos, unsigned char* buffer){
        buffer[(*pos)++] = size;
        buffer[(*pos)++] = id;
        memcpy(buffer+(*pos),data,size);
        *pos += size;
}


