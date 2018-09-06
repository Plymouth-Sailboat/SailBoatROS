//-----------------------------------------------------------------------------------------------
//---------------------------------------COMMENTAIRES--------------------------------------------
// Look at the name of all subscibers
//
// I chose to do a classe for the message to be easyer to check errors in the message
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
#include "iostream"
#include "bitset"
#include "iomanip"
#include "ais_decode_node/Message.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sailrobot_custom_msg/Ais.h" //type message to send Ais infos
#include "sailrobot_custom_msg/AisMultiArray.h"
#include <vector>
using namespace std;
//using namespace ros;
//-----------------------------------------------------------------------------------------------
//-----------------------------------------Global Value---------------------------------------------

string id = "";
string data_payload = "";

//-----------------------------------------------------------------------------------------------
//-----------------------------------------FONCTIONS---------------------------------------------
//Convert a binary to a integer
int bin_to_int(string temp_s)
{
	int i=0;
	for (int x=0; x<temp_s.length(); x++) {
		i=i + ((temp_s[x]-48) * (1<<(temp_s.length()-x-1)));
	}
	return i;
}
//Cut a string depending on a separator
vector<string> split(string str, string sep){
  char* cstr = const_cast<char*>(str.c_str());
  char* current;
  vector<string> arr;
  current = strtok(cstr, sep.c_str());
  while(current != NULL){
    arr.push_back(current);
    current = strtok(NULL, sep.c_str());
  }
  return arr;
}
//function which is calling by subsriber
void ChatBack_data_payload(const std_msgs::String::ConstPtr& msg){
	data_payload = msg->data;
}
//-----------------------------------------------------------------------------------------------
//-----------------------------------MAIN--------------------------------------------------------
int main(int argc, char **argv)
{
//Varible Declaration
	vector<string> ais_string;
	string ais_binary;
	unsigned int x,st=0,fn=0;

	// six bit ascii table
	string six_bit_table[120];
	six_bit_table[48]="000000";
	six_bit_table[49]="000001";
	six_bit_table[50]="000010";
	six_bit_table[51]="000011";
	six_bit_table[52]="000100";
	six_bit_table[53]="000101";
	six_bit_table[54]="000110";
	six_bit_table[55]="000111";
	six_bit_table[56]="001000";
	six_bit_table[57]="001001";
	six_bit_table[58]="001010";
	six_bit_table[59]="001011";
	six_bit_table[60]="001100";
	six_bit_table[61]="001101";
	six_bit_table[62]="001110";
	six_bit_table[63]="001111";
	six_bit_table[64]="010000";
	six_bit_table[65]="010001";
	six_bit_table[66]="010010";
	six_bit_table[67]="010011";
	six_bit_table[68]="010100";
	six_bit_table[69]="010101";
	six_bit_table[70]="010110";
	six_bit_table[71]="010111";
	six_bit_table[72]="011000";
	six_bit_table[73]="011001";
	six_bit_table[74]="011010";
	six_bit_table[75]="011011";
	six_bit_table[76]="011100";
	six_bit_table[77]="011101";
	six_bit_table[78]="011110";
	six_bit_table[79]="011111";
	six_bit_table[80]="100000";
	six_bit_table[81]="100001";
	six_bit_table[82]="100010";
	six_bit_table[83]="100011";
	six_bit_table[84]="100100";
	six_bit_table[85]="100101";
	six_bit_table[86]="100110";
	six_bit_table[87]="100111";

	six_bit_table[96]="101000";
	six_bit_table[97]="101001";
	six_bit_table[98]="101010";
	six_bit_table[99]="101011";
	six_bit_table[100]="101100";
	six_bit_table[101]="101101";
	six_bit_table[102]="101110";
	six_bit_table[103]="101111";
	six_bit_table[104]="110000";
	six_bit_table[105]="110001";
	six_bit_table[106]="110010";
	six_bit_table[107]="110011";
	six_bit_table[108]="110100";
	six_bit_table[109]="110101";
	six_bit_table[110]="110110";
	six_bit_table[111]="110111";
	six_bit_table[112]="111000";
	six_bit_table[113]="111001";
	six_bit_table[114]="111010";
	six_bit_table[115]="111011";
	six_bit_table[116]="111100";
	six_bit_table[117]="111101";
	six_bit_table[118]="111110";
	six_bit_table[119]="111111";

	//Initialisation ROS
	ros::init(argc, argv, "ais_decodeur_node");
	ros::NodeHandle n;

	//Publisher & declaration message
	ros::Publisher chat_ais = n.advertise<sailrobot_custom_msg::AisMultiArray>("Ais_infos", 1000);
	sailrobot_custom_msg::Ais mes_ais;
	sailrobot_custom_msg::AisMultiArray Tab_Ais;

	//Subscriber
	ros::Subscriber sub_data_payload = n.subscribe("Data_Payload", 1000, ChatBack_data_payload);

	ros::Rate loop_rate(20);
	while(ros::ok()){
//Parsing Data Payload Message
		ais_string = split(data_payload, "\n");
		for(size_t compt(0); compt< ais_string.size(); compt++){
	// convert 6 bit string to binary
			ais_binary = "";
			for (x=0; x<ais_string[compt].length(); x++){
				int z,y;
				z = ais_string[compt][x];
				y = (x+1) * 6-5;
				ais_binary.append(six_bit_table[z]);
			}
			if (ais_binary.length() > 137){
	//Decode: conversion binary to number (=> code ASCII)
			string temp_s = ais_binary.substr(0,6);
			int ais_message_type = bin_to_int(temp_s);

			temp_s = ais_binary.substr(6,2);
			int ais_repeat_indicator = bin_to_int(temp_s);

			temp_s = ais_binary.substr(8,30);
			int ais_mmsi = bin_to_int(temp_s);

			temp_s = ais_binary.substr(38,4);
			int ais_navigation_status = bin_to_int(temp_s);

			temp_s = ais_binary.substr(42,8);
			int ais_rate_of_turn = bin_to_int(temp_s);

			temp_s = ais_binary.substr(50,10);
			int ais_speed_over_ground = bin_to_int(temp_s)/10;

			string ais_position_accuracy  = ais_binary.substr(60,1);

			char west = ais_binary[61];
			temp_s = ais_binary.substr(61,28);
			bitset<28> hh(temp_s);
			if (west == '1') {hh.flip();}
			double ais_longitude = hh.to_ulong();;
			ais_longitude = ais_longitude/10000/60;
			if (west == '1') {ais_longitude *= -1;}

			char south = ais_binary[89];
			temp_s = ais_binary.substr(89,27);
			bitset<27> ii(temp_s);
			if (south == '1') {ii.flip();}
			double ais_latitude = ii.to_ulong();
			ais_latitude = ais_latitude/10000/60;
			if (south == '1') {ais_latitude *= -1;}

			temp_s = ais_binary.substr(116,12);
			int ais_course_over_ground = bin_to_int(temp_s);

			temp_s = ais_binary.substr(128,9);
			int ais_true_heading = bin_to_int(temp_s);

	//Fill a message objet to keep it somewhere
			Message message;
			message.setAll(ais_message_type, ais_repeat_indicator, ais_mmsi, ais_navigation_status, ais_rate_of_turn, ais_speed_over_ground, ais_position_accuracy, ais_longitude, ais_latitude, ais_course_over_ground, ais_true_heading);
	//Fill the Ais message
			mes_ais.type = message.getType();
			mes_ais.repeat_indic = message.getRepeat_indic();
			mes_ais.mmsi = message.getMmsi();
			mes_ais.status = message.getNavig_statu();
			mes_ais.rate_of_turn = message.getRate_of_turn();
			mes_ais.speed_over_ground = message.getSpeed();
			mes_ais.position_accuracy = message.getPos_accuracy();
			mes_ais.longitude = message.getLongitude();
			mes_ais.latitude = message.getLatitude();
			mes_ais.course_over_ground = message.getCourse();
			mes_ais.heading = message.getHeading();

	//Fill the message table
			Tab_Ais.data.push_back(mes_ais);
		}

	//Send the table
		chat_ais.publish(Tab_Ais);
		Tab_Ais.data.clear();
	}

	ros::spinOnce();
	loop_rate.sleep();
	}
	return 0;
}
