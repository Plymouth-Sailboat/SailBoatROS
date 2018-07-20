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
	float orientation[4];
	memcpy(orientation,data+2,size-2);
	parsed.orientation.x = orientation[0]; 
        parsed.orientation.y = orientation[1];
        parsed.orientation.z = orientation[2];
        parsed.orientation.w = orientation[3];
	return size;
}

int XGps::parse(unsigned char* data){
        int size = data[0]+2;
        float gps[2];
        memcpy(gps,data+2,size-2);
        parsed.latitude = gps[0];
        parsed.longitude = gps[1];
        return size;
}

int XPose2D::parse(unsigned char* data){
        int size = data[0]+2;
        float orientation[4];
        memcpy(orientation,data+2,size-2);
        return size;
}

int XString::parse(unsigned char* data){
        int size = data[0]+2;
        std::string msg(data+2,data+2+size);
	parsed.data = msg;
        return size;
}

int XFloat32::parse(unsigned char* data){
        int size = data[0]+2;
        float value;
        memcpy(&value,data+2,size-2);
	parsed.data = value;
        return size;
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

