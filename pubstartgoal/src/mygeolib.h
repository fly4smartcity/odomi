// functions that implement TOMA-DARA algorithm
// GPS to meter 
// meter to pixels


#define pi 3.14159265358979323846

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
  float WGS84EarthRadius(float lat)
  { 
  
   // Semi-axes of WGS-84 geoidal reference
   float WGS84_a = 6378137.0;  // Major semiaxis [m]
   float WGS84_b = 6356752.3;  // Minor semiaxis [m]

   float An = WGS84_a*WGS84_a * cos(lat);
   float Bn = WGS84_b*WGS84_b * sin(lat);
   float Ad = WGS84_a * cos(lat);
   float Bd = WGS84_b * sin(lat);

   float radius = sqrt((An*An + Bn*Bn)/(Ad*Ad + Bd*Bd));
   return radius;
  }

    float convGPS(float h_lat_off, float h_lng_off, float distInm, bool is_lat)
  {

//    float lat = deg2rad(45.06708);
//    float lng = deg2rad(7.68803);

    float lat = deg2rad(h_lat_off);
    float lng = deg2rad(h_lng_off);

    float result = 0;    
    
    // Radius of Earth at given latitude
    float radius = WGS84EarthRadius(lat);
    
    // Radius of the parallel at given latitude
    float pradius = radius*cos(lat);

    if(is_lat == true) // if it s a lat calculation 
    {
    float latGps = lat - distInm/radius;
	result = rad2deg(latGps);
    }
    else // or a lng
    {
    float lngGps = lng + distInm/pradius;
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
