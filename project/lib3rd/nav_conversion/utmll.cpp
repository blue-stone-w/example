
#include "navsat_conversions.h"
#include <iostream>
#include <unistd.h>
int main(int argc, char** argv) {

  double lat = 33.75, lng = 113.29;

  double UTMNorthing, UTMEasting;
  std::string UTMZone;

  RobotLocalization::NavsatConversions::LLtoUTM(lat, lng, UTMNorthing, UTMEasting, UTMZone); 
  std::cout<<"UTMNorthing: "<<UTMNorthing<<"; UTMEasting: "<<UTMEasting<<"; UTMZone: "<<UTMZone<<std::endl;
  // pause();
  RobotLocalization::NavsatConversions::UTMtoLL(UTMNorthing,UTMEasting,"49S", lat, lng);
  std::cout<<"lat: "<<lat<<"; lng: "<<lng<<std::endl;
  return 0;
}