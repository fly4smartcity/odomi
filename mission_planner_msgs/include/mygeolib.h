// functions that implement TOMA-DARA algorithm
// GPS to meter 
// meter to pixels

#include <math.h>
#include <string.h>
#include <fstream>
#include <ctime>



namespace mygeolib_tool {

  const double pi = 3.14159265358979323846;
  const double map_ext =  100;
  /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  /*::  This function converts decimal degrees to radians             :*/
  /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  double deg2rad(double deg);

  /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  /*::  This function converts radians to decimal degrees             :*/
  /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  double rad2deg(double rad);


  double distance(double lat1, double lon1, double lat2, double lon2, char unit);

  // from http://stackoverflow.com/questions/238260/how-to-calculate-the-bounding-box-for-a-given-lat-lng-location
  double WGS84EarthRadius(double lat);

    double convGPS(double h_lat_off, double h_lng_off, double distInm, bool is_lat);


	int roundUp(int numToRound, int multiple);

};