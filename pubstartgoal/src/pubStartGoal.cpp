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
#include "nav_msgs/OccupancyGrid.h"
#include <math.h>
#include <string.h>
#include <fstream>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>



#define pi 3.14159265358979323846

using namespace cv;
using namespace std;

 geometry_msgs::PoseStamped goal;
 geometry_msgs::PoseStamped start;
 
 ros::Publisher goal_pub;
 ros::Publisher start_pub;
 ros::Publisher wp_pub;
 ros::Publisher map_pub;


 cv::Mat mappa;
 Mat crop ;

 int map_width;
 int map_height;

 double home_lat, goal_lat;
 double home_lng, goal_lng;

 double sgn_lat ; // sgn is 1 or -1
 double sgn_lng ;

 int paddingx = 200; // multiple of 4 !!!
 int paddingy = 200;


 int ready2odomi = 0 ; 
 bool pub = false;

 float KV = 1.0706; // [px/m]
 float KO = 1.06853; // [px/m]

 typedef vector <Point> Polygon;


  void publishMap(Mat m) // publish to odomi_path_planner the map
  {
    

    std::vector<int8_t> map;

    nav_msgs::OccupancyGrid pubMap;
    
    pubMap.header.stamp = ros::Time::now();
    pubMap.info.map_load_time = ros::Time::now();
    pubMap.header.frame_id = "map";
    pubMap.info.width = m.cols;
    pubMap.info.height = m.rows;


    for(int j=m.rows;j > 0 ;j--)
      for(int i=0;i < m.cols;i++)
      {

        if(m.at<unsigned char>(j,i)>100)
          map.push_back(0);
        else
          map.push_back(100);
        
      }

      pubMap.data = map;

      
      map_pub.publish(pubMap);
      
  }


  void callServiceOpendata(String lab) {

  String whatcall = lab;
  // call open data service  for the bounding box
  ros::NodeHandle n;
  ros::ServiceClient bbox_service = n.serviceClient<open_data_msg::BoundingBox>("bounding_box");

  open_data_msg::BoundingBox srv;

  srv.request.label = whatcall.c_str();
  srv.request.type  = 1;
  srv.request.bounding_box.points.resize(2);

    

  srv.request.bounding_box.points[0].x = goal_lng; 
  srv.request.bounding_box.points[0].y = goal_lat;
  srv.request.bounding_box.points[1].x = home_lng;
  srv.request.bounding_box.points[1].y = home_lat;


 // call service open data for buildings  
  if (bbox_service.call(srv))
  {
  if(srv.response.resp)
      ROS_INFO("Result: %s true called bbox %f %f %f %f",whatcall.c_str(), goal_lat, goal_lng, home_lat, home_lng);
  else          
     ROS_INFO("Result: false");
  }
  else
  {
    ROS_ERROR("Failed to call service edifici bounding_box");
    
  }


  }

  void subGoal(const mission_planner_msgs::CoordinateArray::ConstPtr& wp) // subscribe goal
  {
  int s = wp->waypoint.size(); 	  

  if( s!= 0){
  ROS_INFO("Subscribing Waypoints [%d]", wp->waypoint.size());
  
  goal_lat = wp->waypoint[s-1].latitude;
  goal_lng = wp->waypoint[s-1].longitude;

  double disty = distance(home_lat,home_lng,goal_lat, home_lng,'K')*1000; // in m
  double distx = distance(home_lat,home_lng,home_lat,goal_lng,'K')*1000;

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


  callServiceOpendata("alberate");
  callServiceOpendata("edifici");
  

  ready2odomi = 0;
    
  }


  }

  void subWp(const mission_planner_msgs::CoordinateArray::ConstPtr& wayp) // publish towards the MP the array of waypoints
  {
  
  mission_planner_msgs::CoordinateArray msg;
  mission_planner_msgs::Coordinate wp;

  //conversion from pix to GPS
  
  double lat_m;
  double lng_m;

  for(int i = 0 ; i < wayp->waypoint.size(); i++){

  lat_m = wayp->waypoint[i].latitude; //distance lat in m
  lng_m = wayp->waypoint[i].longitude; //distance lng in m

  wp.latitude = convGPS(home_lat,home_lng,sgn_lat*lat_m, true);
  wp.longitude = convGPS(home_lat,home_lng,sgn_lng*lng_m, false);

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
 if(ready2odomi == 0) {

 ROS_INFO("Subscribing Sensor Packet lat [%f] long [%f]", spacket->h_latit , spacket->h_longit);
 pub = true;
 ready2odomi = 1;
 
 }

  float Plat = spacket->h_latit;
  float Plng = spacket->h_longit;

  home_lat = spacket->h_latit;
  home_lng = spacket->h_longit;

  start.pose.position.x =  0; 
  start.pose.position.y =  0;  
  
 }
}


void subOd(const open_data_msg::DataConstPtr& opendata) // subscribe open data 
{
    	int ind = 0 ;
      double origin_lat;
      double origin_lng;

       sgn_lat = (goal_lat - home_lat)/sqrt((goal_lat - home_lat)*(goal_lat - home_lat)); // sgn is 1 or -1
       sgn_lng = (goal_lng - home_lng)/sqrt((goal_lng - home_lng)*(goal_lng - home_lng));

      // ROS_INFO("sgn lat %f lng %f",sgn_lat,sgn_lng);
  // put map origin according to the goal position
  // if it s 1 quad
   origin_lat = convGPS(home_lat, home_lng, -1*sgn_lat*paddingy,true);
   origin_lng = convGPS(home_lat, home_lng, -1*sgn_lng*paddingx,false);
	
    String attributes_key = opendata->data[0].label.c_str();
    ROS_INFO("labels : %s", opendata->data[0].label.c_str());
    if(attributes_key.compare("edifici") ||  attributes_key.compare("alberate")){
  	for(int j = 0; j < opendata->data.size(); j++) // for su opendata
	{ 


		float height = atof(opendata->data[j].attributes[0].value.c_str());
		if( height > 1.0 || attributes_key.compare("alberate") == 0 ) // height>1 -> is not a street 
		{
			Polygon poly;

		  	for (int i=0; i < opendata->data[j].area.points.size(); i++) // number of points into the polygon
		  	{
		  		if(opendata->data[j].area.points[i].x != 0 && opendata->data[j].area.points[i].y != 0)
		  		{	
					double distpointy = distance(origin_lat,origin_lng,opendata->data[j].area.points[i].x,origin_lng ,'K')*1000 ; // in m reference to the origin
					double distpointx = distance(origin_lat,origin_lng,origin_lat,opendata->data[j].area.points[i].y,'K')*1000  ;		
					

					poly.push_back(Point(distpointx, distpointy));
					
					// //if(distpointx < paddingx || distpointy < paddingy );
					// 	ROS_INFO("Height: %f, Points are in m x %f y %f and gps %f %f home is %f %f, map dimensions: %d %d", height,
					// 	distpointx, distpointy,opendata->data[j].area.points[i].x,opendata->data[j].area.points[i].y,origin_lat,origin_lng,
					// 	map_width,map_height);
		  			
	  			}
	  			else // draw poly
	  			{
            if(attributes_key.compare("edifici") == 0)
            {
					vector<Point> tmp = poly;
					const Point* elementPoints[1] = { &tmp[0] };
					int numberOfPoints = (int)tmp.size();
					fillPoly (mappa, elementPoints, &numberOfPoints, 1, 0);
					polylines(mappa, elementPoints, &numberOfPoints, 1, 0, 0, 0, 1);
					/*
					for(int k=0; k < numberOfPoints;k++)
						circle(mappa,poly[k],2,127,-1);*/
					   }

            if(attributes_key.compare("alberate") == 0){

              vector<Point> tmp = poly;
              const Point* elementPoints[1] = { &tmp[0] };
              circle(mappa,poly[0],4,0,-1);
            }

					poly.clear();
	  			}
	  		}
              if(attributes_key.compare("edifici")  == 0)
              {
              
              vector<Point> tmp = poly;
              const Point* elementPoints[1] = { &tmp[0] };
              int numberOfPoints = (int)tmp.size();
              fillPoly (mappa, elementPoints, &numberOfPoints, 1, 0);
              polylines(mappa, elementPoints, &numberOfPoints, 1, 0, 0, 0, 1);
              /*
              for(int k=0; k < numberOfPoints;k++)
                circle(mappa,poly[k],2,127,-1);*/
               }

               if(attributes_key.compare("alberate") == 0){

                vector<Point> tmp = poly;
                const Point* elementPoints[1] = { &tmp[0] };
                circle(mappa,poly[0],4,0,-1);
              }      
 
			poly.clear();


	  	 
	  	}
  	}
  }
	
	Rect roi(paddingx,paddingy,mappa.cols - paddingx*2,mappa.rows - paddingy*2);
  crop = mappa(roi).clone();


	if(attributes_key.compare("edifici") == 0)
  {

  // add LTE signal
//
  Mat map_lte = cv::Mat(Size(mappa.cols - paddingx*2,mappa.rows - paddingy*2),CV_8UC1); // same as cropped map
  map_lte.setTo(255);

  Polygon polyLte;
  int ii = 0 ;

  // read from text lte-signal.txt
  String line;
  std::vector<double>  lat_read;
  std::vector<double>  lng_read;
  std::vector<double>  lte_signal;

  double lte_threshold = 5;

  ifstream myfile ("/home/sgabello/catkin_ws/src/pubstartgoal/src/lte-signal.txt");
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      ROS_INFO("%s", line.c_str());

      if((ii % 3) == 0) {
        lat_read.push_back(atof(line.c_str()));
      }

      if((ii % 3) == 1) lng_read.push_back(atof(line.c_str()));

      if((ii % 3) == 2) lte_signal.push_back(atof(line.c_str()));

      ii++;
    }
    myfile.close();
  }

  else ROS_INFO("Unable to open file"); 

  int miao = 0 ;
  for (int i=0; i < (ii+1)/3 ; i++)
    {
  //ROS_INFO("bau ! %d signal %f lat %f home lat %f lng %f home lng %f",i, lte_signal[i], lat_read[i],home_lat, lng_read[i],home_lng);        
  // compare GPS - LTE signal to BBox
  // lat_read[i] >= home_lat && lat_read[i] <= goal_lat && lng_read[i] >= home_lng && lng_read[i] <= goal_lng && 

  double min_lat, min_lng;
  double max_lat, max_lng;

  if(home_lat<goal_lat)
  {
    min_lat=home_lat;
    max_lat=goal_lat;
  }
  else
   {
     min_lat=goal_lat;
     max_lat= home_lat;
   }
  if(home_lng<goal_lng)
  {
    min_lng=home_lng;
    max_lng=goal_lng;
  }
  else
   {
    min_lng=goal_lng;  
    max_lng=home_lng;
    }

    if(lte_signal[i] <= lte_threshold  && lat_read[i] >= min_lat && lat_read[i] <= max_lat && lng_read[i] >= min_lng && lng_read[i] <= max_lng){ // gps read within the bbox and lte signal is less than threshold
      
      double y = distance(origin_lat,origin_lng,lat_read[i],origin_lng,'K')*1000 ; // in m reference to the origin
      double x = distance(origin_lat,origin_lng,origin_lat,lng_read[i],'K')*1000 ;   
                
      circle(map_lte,Point(x,y),50,0,-1);

      miao++;

      }

    }
      if(miao != 0) {

      imwrite("/home/sgabello/catkin_ws/src/pubstartgoal/src/lte_map.pgm", map_lte);
      
      bitwise_and(map_lte, crop,crop);              

      ROS_INFO("Map lte added");

      }

      
    if(imwrite("/home/sgabello/catkin_ws/src/pubstartgoal/src/odomimap_edifici.pgm", crop)) // save for visualization
    {
		

    ROS_INFO("Map buildings + tree created ");
    publishMap(crop.clone());

    }
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
  map_pub = n.advertise< nav_msgs::OccupancyGrid>("/map", 1,false);


  ros::Subscriber goal_mp = n.subscribe<mission_planner_msgs::CoordinateArray>("/gui_waypoints", 100, subGoal);
  ros::Subscriber start_mp = n.subscribe<mission_planner_msgs::SensorPacket>("/feedback", 1, subStart);
  ros::Subscriber wp_mp = n.subscribe<mission_planner_msgs::CoordinateArray>("/2mp_wp", 1, subWp);
  ros::Subscriber od = n.subscribe<open_data_msg::Data>("/opendata", 100, subOd);
  
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
