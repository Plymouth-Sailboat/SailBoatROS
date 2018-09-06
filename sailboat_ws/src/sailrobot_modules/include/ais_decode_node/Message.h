#include <iostream>
#include <string>

class Message
{
public:
  Message(); // Constructeur
  void Show() const; // Affichage
  //setter global
  void setAll(int type, int repeat_indic, int mmsi, int navig_statu, int rate_of_turn, int speed, std::string pos_accuracy, double longitude, double latitude, int course, int heading);
  //setter
  void setType(int type);
  void setRepeat_indic(int repeat_indic);
  void setMmsi(int mmsi);
  void setNavig_statu(int navig_statu);
  void setRate_of_turn(int rate_of_turn);
  void setSpeed(int speed);
  void setPos_accuracy(std::string pos_accuracy);
  void setLongitude(double longitude);
  void setLatitude(double latitude);
  void setCourse(int course);
  void setHeading(int heading);
  //getter
  int getType() const;
  int getRepeat_indic() const;
  int getMmsi() const;
  int getNavig_statu() const;
  int getRate_of_turn() const;
  int getSpeed() const;
  std::string getPos_accuracy() const;
  double getLongitude() const;
  double getLatitude() const;
  int getCourse() const;
  int getHeading() const;

private:
  int m_type;
  int m_repeat_indic;
  int m_mmsi;
  int m_navig_statu;
  int m_rate_of_turn;
  int m_speed;
  std::string m_pos_accuracy;
  double m_longitude;
  double m_latitude;
  int m_course;
  int m_heading;
};
