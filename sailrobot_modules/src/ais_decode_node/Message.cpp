#include <iostream>
#include <string>
#include "ais_decode_node/Message.h"
#include "bitset"
#include "iomanip"

using namespace std;

Message::Message(): m_type(0), m_repeat_indic(0), m_mmsi(0), m_navig_statu(0),
m_rate_of_turn(0), m_speed(0), m_pos_accuracy(""), m_longitude(0), m_latitude(0), m_course(0), m_heading(0)
{
}

void Message::Show() const
{
  cout<<"\nmessage type = \t\t"<< m_type;
	cout<<"\nrepeat indicator = \t"<< m_repeat_indic;
	cout<< setfill('0')<< "\nMMSI = \t\t\t" << setw(9) << m_mmsi;
	cout<<"\nnavagation status = \t"<< m_navig_statu;
	cout<<"\nrate of turn = \t\t"<< m_rate_of_turn;
	cout<<"\nspeed over ground = \t"<< m_speed;
	cout<<"\nposition accuracy = \t"<< m_pos_accuracy;
	cout<< fixed << "\nlongitude = \t\t"<< m_longitude;
	cout<< fixed << "\nlatitude = \t\t"<< m_latitude;
	cout<<"\ncourse over ground = \t"<< m_course;
	cout<<"\ntrue heading = \t\t"<< m_heading;
}

void Message::setAll(int type, int repeat_indic, int mmsi, int navig_statu, int rate_of_turn, int speed, string pos_accuracy, double longitude, double latitude, int course, int heading){
  m_type = type;
  m_repeat_indic = repeat_indic;
  m_mmsi = mmsi;
  m_navig_statu = navig_statu;
  m_rate_of_turn = rate_of_turn;
  m_speed = speed;
  m_pos_accuracy = pos_accuracy;
  m_longitude = longitude;
  m_latitude = latitude;
  m_course = course;
  m_heading = heading;
}
//setter
void Message::setType(int type){ m_type = type; }
void Message::setRepeat_indic(int repeat_indic){ m_repeat_indic = repeat_indic; }
void Message::setMmsi(int mmsi){ m_mmsi = mmsi; }
void Message::setNavig_statu(int navig_statu){ m_navig_statu = navig_statu; }
void Message::setRate_of_turn(int rate_of_turn){ m_rate_of_turn = rate_of_turn; }
void Message::setSpeed(int speed){ m_speed = speed; }
void Message::setPos_accuracy(string pos_accuracy){ m_pos_accuracy = pos_accuracy; }
void Message::setLongitude(double longitude) { m_longitude = longitude; }
void Message::setLatitude(double latitude){ m_latitude = latitude; }
void Message::setCourse(int course){ m_course = course; }
void Message::setHeading(int heading){ m_heading = heading; }
//getter
int Message::getType() const { return m_type; }
int Message::getRepeat_indic() const { return m_repeat_indic; }
int Message::getMmsi() const { return m_mmsi; }
int Message::getNavig_statu() const { return m_navig_statu; }
int Message::getRate_of_turn() const { return m_rate_of_turn; }
int Message::getSpeed() const { return m_speed; }
string Message::getPos_accuracy() const { return m_pos_accuracy; }
double Message::getLongitude() const { return m_longitude; }
double Message::getLatitude() const { return m_latitude; }
int Message::getCourse() const { return m_course; }
int Message::getHeading() const { return m_heading; }
