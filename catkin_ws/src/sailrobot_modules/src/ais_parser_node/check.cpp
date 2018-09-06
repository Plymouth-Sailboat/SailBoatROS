#include "ais_parser_node/check.h"
using namespace std;
char ahextobin( char *c )
{
    if( (*c >= '0') && (*c <= '9') )
    {
        return *c - '0';
    } else if( (*c >= 'A') && (*c <= 'F') ) {
        return *c - ('A'-10);
    } else if( (*c >= 'a') && (*c <= 'f') ) {
        return *c - ('a'-10);
    }

    return -1;
}
unsigned char asc_to_hex(char *p){
	unsigned char v = 0;
	int i = 0;
	v = ahextobin(p) << 4;
	v += ahextobin(p+1);
	printf(" CHECKSUM READ -> %#08X \n",v);
	return v;
}

int nmea_checksum(unsigned char* checksum, char *p){

        int i=0;
        *checksum = 0;
	p++;
	while( (*p != 0) && (*p != '*') && !((*p == '!') || (*p == '$')) )
	{
		*checksum ^= *p;
		p++;
	}
	printf("CHECKSUM CALCULATED -> %#08X \n ", *checksum);   // gives 0x000007
	
	return 0;
}


int check_nmea_checksum(char* read, char *p){

        //variable stockant la valeur calculée du checksum
        unsigned char calcSum = 0;
        unsigned char* pcalcSum = &calcSum;

	nmea_checksum(pcalcSum,p);	

	return (asc_to_hex(read) == calcSum);
}
