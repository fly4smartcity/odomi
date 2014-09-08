// functions that implement TOMA-DARA algorithm
// GPS to meter 
// meter to pixels

// #include <math.h>
// #include <string.h>
// #include <fstream>
// #include <ctime>
 #include "mygeolib.h"
// #define pi 3.14159265358979323846
// #define map_ext 100

namespace mygeolib_tool {

  /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  /*::  This function converts decimal degrees to radians             :*/
  /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  double deg2rad(double deg) {
  return (deg * pi / 180);
  }

  /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  /*::  This function converts radians to decimal degrees             :*/
  /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  double rad2deg(double rad) {
  return (rad * 180 / pi);
	}


  double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
  double theta, dist;
  theta = lon1 - lon2;

  dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515;
  switch(unit) {
    case 'M':
      break;
    case 'K':
      dist = dist * 1.609344;
      break;
    case 'N':
      dist = dist * 0.8684;
      break;
  }
  return dist;
  }

  // from http://stackoverflow.com/questions/238260/how-to-calculate-the-bounding-box-for-a-given-lat-lng-location
  double WGS84EarthRadius(double lat)
  { 
  


   // Semi-axes of WGS-84 geoidal reference
   double WGS84_a = 6378137.0;  // Major semiaxis [m]
   double WGS84_b = 6356752.3;  // Minor semiaxis [m]

   double An = WGS84_a*WGS84_a * cos(lat);
   double Bn = WGS84_b*WGS84_b * sin(lat);
   double Ad = WGS84_a * cos(lat);
   double Bd = WGS84_b * sin(lat);

   double radius = sqrt((An*An + Bn*Bn)/(Ad*Ad + Bd*Bd));
   return radius;
  }

    double convGPS(double h_lat_off, double h_lng_off, double distInm, bool is_lat)
  {

    double lat = deg2rad(h_lat_off);
    double lng = deg2rad(h_lng_off);

    double result = 0;    
    
    // Radius of Earth at given latitude
    double radius = WGS84EarthRadius(lat);
    
    // Radius of the parallel at given latitude
    double pradius = radius*cos(lat);

    if(is_lat == true) // if it s a lat calculation 
    {
    double latGps = lat + distInm/radius;
	result = rad2deg(latGps);
    }
    else // or a lng
    {
    double lngGps = lng + distInm/pradius;
	result = rad2deg(lngGps);
   }

   return result;
  }


	int roundUp(int numToRound, int multiple) 
	{ 
	if(multiple == 0) 
	{ 
	return numToRound; 
	} 

	int remainder = numToRound % multiple;
	if (remainder == 0)
	return numToRound;
	return numToRound + multiple - remainder;
	}

};