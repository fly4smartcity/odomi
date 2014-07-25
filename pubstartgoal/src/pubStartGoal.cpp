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
#include "mygeolib.h"
#include "open_data_msg/BoundingBox.h"
#include "open_data_msg/Data.h"
#include <cstdlib>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include <iostream>
#define pi 3.14159265358979323846

using namespace cv;
using namespace std;

 geometry_msgs::PoseStamped goal;
 geometry_msgs::PoseStamped start;
 
 ros::Publisher goal_pub;
 ros::Publisher start_pub;
 ros::Publisher wp_pub;

 cv::Mat mappa;

 float map_width;
 float map_height;

 double home_lat;
 double home_lng;

 int paddingx = 200; // multiple of 4 !!!
 int paddingy = 200;


 int j = 0 ; 
 bool pub = false;

 float KV = 1.0706; // [px/m]
 float KO = 1.06853; // [px/m]

 typedef vector <Point> Polygon;

  void subGoal(const mission_planner_msgs::CoordinateArray::ConstPtr& wp)
  {
  int s = wp->waypoint.size();
  float Plat;
  float Plng; 	  

  if( s!= 0){
  ROS_INFO("Subscribing Waypoints [%d]", wp->waypoint.size());
  
  Plat = wp->waypoint[s-1].latitude;
  Plng = wp->waypoint[s-1].longitude;

  double disty = distance(home_lat,home_lng,Plat, home_lng,'K')*1000; // in m
  double distx = distance(home_lat,home_lng,home_lat,Plng,'K')*1000;

  map_width =  roundUp(distx , 4);
  map_height = roundUp(disty , 4);

  goal.pose.position.x = distx;  // px = m * px/m
  goal.pose.position.y = disty;
  
  // create map px = m



  mappa =  cv::Mat(Size(map_width + paddingy*2,map_height+ paddingy*2),CV_8UC1);
  mappa.setTo(255);
  
  ROS_INFO("Publishing target %f %f and home position %f %f \n",goal.pose.position.x, goal.pose.position.y, start.pose.position.x, start.pose.position.y );
  ROS_INFO("Distx %f disty %f start_lat %f start_lng %f",distx,disty, start.pose.position.x,start.pose.position.y );
  pub = true;

  goal_pub.publish(goal);
  }

  // call open data service  for the bounding box
  ros::NodeHandle n;
  ros::ServiceClient bbox_service = n.serviceClient<open_data_msg::BoundingBox>("bounding_box");

  open_data_msg::BoundingBox srv;

  srv.request.label = "edifici";
  srv.request.bounding_box.points.resize(2);

	

	srv.request.bounding_box.points[0].x = Plng; 
	srv.request.bounding_box.points[0].y = Plat;
	srv.request.bounding_box.points[1].x = home_lng;
	srv.request.bounding_box.points[1].y = home_lat;


 // call service open data for buildings  
  if (bbox_service.call(srv))
  {
	if(srv.response.resp)
	    ROS_INFO("Result: edifici: true called bbox %f %f %f %f", Plat, Plng, home_lat, home_lng);
	else			    
	   ROS_INFO("Result: false");
  }
  else
  {
    ROS_ERROR("Failed to call service edifici bounding_box");
    
  }


 // call service open data for trees  
/*
  srv.request.label = "alberate";
  if (bbox_service.call(srv))
  {
	if(srv.response.resp)
	    ROS_INFO("Result: aleberate: true called bbox %f %f %f %f", Plat, Plng, home_lat, home_lng);
	else			    
	   ROS_INFO("Result: false");
  }
  else
  {
    ROS_ERROR("Failed to call service alberate bounding_box");
    
  }
*/
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

  lat_m = wayp->waypoint[i].latitude; //distance lat in m
  lng_m = wayp->waypoint[i].longitude; //distance lng in m

  wp.latitude = convGPS(home_lat,home_lng,lat_m, true);
  wp.longitude = convGPS(home_lat,home_lng,lng_m, false);

  msg.waypoint.push_back(wp);
  ROS_INFO("publishing wp lat %f , %f  and wp in km %f , %f \n", wp.latitude, wp.longitude, lat_m, lng_m);

  }

  //mo provo con la home in px se diventa gps
/*
  lat_m =   463.056  /( KV);
  lng_m =   470.095 / (KO);

  wp.latitude = convGPS(h_lat_off,h_lng_off,lat_m, true);
  wp.longitude = convGPS(h_lat_off,h_lng_off,lng_m, false);

  ROS_INFO("home wp lat %f , %f  and wp in km %f , %f \n", wp.latitude, wp.longitude, lat_m, lng_m);

*/
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

  home_lat = spacket->h_latit;
  home_lng = spacket->h_longit;

  //float disty = distance(h_lat_off,h_lng_off,Plat,h_lng_off ,'K')*1000; // in m

  //float distx = distance(h_lat_off,h_lng_off,h_lat_off,Plng,'K')*1000;

  start.pose.position.x =  0; 
  start.pose.position.y =  0;

  if(j == 1){
  //ROS_INFO("Publishing start_lat %f start_lng %f Plat %f Plng %f Distx %f disty %f \n",start.pose.position.x,start.pose.position.y,Plat,Plng,distx,disty);
  j++;
  }
    start_pub.publish(start);
  
 }
}

void subOd(const open_data_msg::DataConstPtr& opendata)
{
    	int ind = 0 ;
    
	double origin_lat = convGPS(home_lat, home_lng, -1*paddingy,true);
	double origin_lng = convGPS(home_lat, home_lng, -1*paddingx,false);


  	for(int j = 0; j < opendata->data.size(); j++) // for su opendata
	{ 


		float height = atof(opendata->data[j].attributes[0].value.c_str());
		if( height > 1.0) // height>1 -> is not a street 
		{
			Polygon poly;
			
		  	for (int i=0; i < opendata->data[j].area.points.size(); i++) // number of points into the polygon
		  	{
		  		if(opendata->data[j].area.points[i].x != 0 && opendata->data[j].area.points[i].y != 0)
		  		{	
					double distpointy = distance(origin_lat,origin_lng,opendata->data[j].area.points[i].x,origin_lng ,'K')*1000 ; // in m
					double distpointx = distance(origin_lat,origin_lng,origin_lat,opendata->data[j].area.points[i].y,'K')*1000  ;		
					/*
					if(distpointx >=  map_width)
						distpointx =  map_width -1 ;
					if(distpointy >=  map_height)
						distpointy =  map_height -1;

					if(distpointx <  0)
						distpointx =  0;
					if(distpointy <  0)
						distpointy =  0;
					*/

					poly.push_back(Point(distpointx, distpointy));
					
					if(distpointx < paddingx || distpointy < paddingy )
						ROS_INFO("Height: %f, Points are in m x %f y %f and gps %f %f home is %f %f, map dimensions: %f %f", height,
						distpointx, distpointy,opendata->data[j].area.points[i].x,opendata->data[j].area.points[i].y,origin_lat,origin_lng,
						map_width,map_height);
		  			
	  			}
	  			else // draw poly
	  			{
					vector<Point> tmp = poly;
					const Point* elementPoints[1] = { &tmp[0] };
					int numberOfPoints = (int)tmp.size();
					fillPoly (mappa, elementPoints, &numberOfPoints, 1, 0);
					polylines(mappa, elementPoints, &numberOfPoints, 1, 0, 0, 0, 1);
					/*
					for(int k=0; k < numberOfPoints;k++)
						circle(mappa,poly[k],2,127,-1);*/
					
					poly.clear();
	  			}
	  		}
			vector<Point> tmp = poly;
			const Point* elementPoints[1] = { &tmp[0] };
			int numberOfPoints = (int)tmp.size();
			fillPoly (mappa, elementPoints, &numberOfPoints, 1, 0);
			polylines(mappa, elementPoints, &numberOfPoints, 1, 0, 0, 0, 1);
/*
			for(int k=0; k < numberOfPoints;k++)
				circle(mappa,poly[k],2,127,-1);
			*/
			poly.clear();


	  	 
	  	}
  	}
  
	//save to file
	
	Rect roi(paddingx,paddingy,mappa.cols - paddingx*2,mappa.rows - paddingy*2);
	Mat crop = mappa(roi).clone();


	if(imwrite("/tmp/odomimap.pgm", crop))
		ROS_INFO("Map crop created");
if(imwrite("/tmp/odomimap_raw.pgm", mappa))
		ROS_INFO("Map created 2");

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
  ros::Subscriber od = n.subscribe<open_data_msg::Data>("/opendata", 1, subOd);
  
  ros::Rate loop_rate(10);
 
  

  //goal
  goal.header.frame_id = 'g';
 
  
  start.header.frame_id = 's';
  
  
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
