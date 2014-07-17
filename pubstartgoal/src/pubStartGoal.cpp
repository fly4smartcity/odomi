#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include <sstream>
#include <mission_planner_msgs/CoordinateArray.h>
#include <mission_planner_msgs/SensorPacket.h>
#include <ctime>
#include <math.h>
#include <float.h>


///////// BBBBBBBBBBBOMBER

#define pi 3.14159265358979323846

 geometry_msgs::PoseStamped goal;
 geometry_msgs::PoseStamped start;
 
 ros::Publisher goal_pub;
 ros::Publisher start_pub;
 ros::Publisher wp_pub;

 float h_lat_off = 45.06708;
 float h_lng_off = 7.688032;
 
 int j = 0 ; 
 bool pub = false;

 float KV = 1.0706; // [px/m]
 float KO = 1.06853; // [px/m]


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
  return (dist);
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

    float convGPS(float distInm, bool is_lat)
  {

    // latitudeInDegrees = h_lat_off 
    // longitudeInDegrees = h_lng_off 

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

  void subGoal(const mission_planner_msgs::CoordinateArray::ConstPtr& wp){
  int s = wp->waypoint.size();

  if( s!= 0){
  ROS_INFO("Subscribing Waypoints [%d]", wp->waypoint.size());
  
  float Plat = wp->waypoint[s-1].latitude;
  float Plng = wp->waypoint[s-1].longitude;

  float disty = distance(h_lat_off,h_lng_off,Plat, h_lng_off,'K')*1000; // in m
  float distx = distance(h_lat_off,h_lng_off,h_lat_off,Plng,'K')*1000;

  goal.pose.position.x = distx*KV;  // px = m * px/m
  goal.pose.position.y = disty*KO;
  
  ROS_INFO("Publishing target %f %f and home position %f %f \n",goal.pose.position.x, goal.pose.position.y, start.pose.position.x, start.pose.position.y );
  ROS_INFO("Distx %f disty %f start_lat %f start_lng %f",distx,disty, start.pose.position.x,start.pose.position.y );
  pub = true;

  goal_pub.publish(goal);
  }

  }

  void subWp(const mission_planner_msgs::CoordinateArray::ConstPtr& wayp) // publish towards the MP the array of waypoints
  {
  
  mission_planner_msgs::CoordinateArray msg;
  mission_planner_msgs::Coordinate wp;

  //conversion from pix to GPS

   /*
  float lat_gps = lat_m*Kg_lat + h_lat_off;
  float lng_gps = lng_m*Kg_lng + h_lng_off;
   */
  
  float lat_m;
  float lng_m;

  for(int i = 0 ; i < wayp->waypoint.size(); i++){

  lat_m = wayp->waypoint[i].latitude /( KV); //distance lat in m
  lng_m = wayp->waypoint[i].longitude / (KO); //distance lng in m

  /*
  float lat_gps = lat_m/110.54;
  float lat_rad = deg2rad(lat_gps);
  float lng_gps = lng_m/(111.32*cos(lat_rad)); */

  wp.latitude = convGPS(lat_m, true);
  wp.longitude = convGPS(lng_m, false);

  msg.waypoint.push_back(wp);
  ROS_INFO("publishing wp lat %f , %f  and wp in km %f , %f \n", wp.latitude, wp.longitude, lat_m, lng_m);

  }

  //mo provo con la home in px se diventa gps

  lat_m =   463.056  /( KV);
  lng_m =   470.095 / (KO);

  wp.latitude = convGPS(lat_m, true);
  wp.longitude = convGPS(lng_m, false);

  ROS_INFO("home wp lat %f , %f  and wp in km %f , %f \n", wp.latitude, wp.longitude, lat_m, lng_m);


  wp_pub.publish(msg);


  }

  

  void subStart(const mission_planner_msgs::SensorPacketConstPtr& spacket)
  {
  
 if(spacket->h_latit != 0 && spacket->h_longit != 0)
 {
 if(j == 0) {

 ROS_INFO("Subscribing Sensor Packet lat [%f] long [%f]", spacket->h_latit , spacket->h_longit);
 pub = true;
 j++;
 
 }

  float Plat = spacket->h_latit;
  float Plng = spacket->h_longit;

  float disty = distance(h_lat_off,h_lng_off,Plat,h_lng_off ,'K')*1000; // in m

  float distx = distance(h_lat_off,h_lng_off,h_lat_off,Plng,'K')*1000;

  start.pose.position.x =  distx*KV; 
  start.pose.position.y =  disty*KO;

  if(j == 1){
  ROS_INFO("Publishing start_lat %f start_lng %f Plat %f Plng %f Distx %f disty %f \n",start.pose.position.x,start.pose.position.y,Plat,Plng,distx,disty);
  j++;
  }
    start_pub.publish(start);
  
 }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "publisher_goal_and_start");
   
  int i = 0;
  
   
  ros::NodeHandle n;

  goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 2,false);
  start_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 2,false);
  wp_pub    = n.advertise<mission_planner_msgs::CoordinateArray>("/waypoints", 2,false);

  ros::Subscriber goal_mp = n.subscribe<mission_planner_msgs::CoordinateArray>("/gui_waypoints", 1, subGoal);
  ros::Subscriber start_mp = n.subscribe<mission_planner_msgs::SensorPacket>("/feedback", 1, subStart);
  ros::Subscriber wp_mp = n.subscribe<mission_planner_msgs::CoordinateArray>("/2mp_wp", 1, subWp);
  
  ros::Rate loop_rate(10);
 
  

  //goal
  goal.header.frame_id = 'g';
 
  
  //goal.pose.position.x = 208;  
  //goal.pose.position.y = 139;
  //start
  start.header.frame_id = 's';
 
 // start.pose.position.x = 30;  
 // start.pose.position.y = 50;
  //ros::Rate loop_rate(0.1);
  //ros::Duration onesec(1,0);

  
  
//&& i <= 1
  while(ros::ok()){
  
  if (pub == true){

  pub = false;
  ROS_INFO("...\n" );
  }

  ros::spinOnce();
  loop_rate.sleep();

  }

  return 0;
}
