#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point32.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include <sstream>
#include <mission_planner_msgs/CoordinateArray.h>
#include <mission_planner_msgs/SensorPacket.h>
#include <ctime>
#include <see0d/buildings.h>

#include <float.h>
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

#include <odomi_path_planner/mygeolib.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/navdata_gps.h>
#include <geometry_msgs/PoseWithCovariance.h>

using namespace cv;
using namespace std;
using namespace mygeolib_tool;

class See0d
{
private:

    ros::NodeHandle n;
    ros::Publisher map_pub;
    ros::Publisher edi_pub,tree_pub, camera;
    ros::Subscriber visOD, parrotAdapter;
    ros::Subscriber start_mp;

    ros::ServiceClient bbox_service;



    cv::Mat mappa;
    Mat crop ;

    geometry_msgs::PoseStamped start;

    int map_width;
    int map_height;

    double home_lat,goal_lat;
    double home_lng, goal_lng;

    double BB_home_lat,BB_home_lng, BB_goal_lat, BB_goal_lng;

    double sgn_lat ; // sgn is 1 or -1
    double sgn_lng ;

    typedef vector <Point> Polygon;


public:


  double origin_lat;
  double origin_lng;
  double map_ext;

  open_data_msg::BoundingBox srv; // service open data
  see0d::buildings buildings;

  geometry_msgs::Polygon messageTree;

  int paddingx; // multiple of 4 !!!
  int paddingy;

  bool call_serviceod; 


  See0d()
  {


    call_serviceod = true;

    ros::NodeHandle private_nh("~");
    //private_nh.param("bounding_box_extension", map_ext, 200.00);
    map_ext = 200;

    paddingx = 300; // multiple of 4 !!!
    paddingy = 300;


    //visOD = n.subscribe<open_data_msg::Data>("/opendata", 100, &See0d::subODforVisualization2D, this);
    visOD = n.subscribe<open_data_msg::Data>("/opendata", 100, &See0d::subODforVisualization3D, this);
    start_mp = n.subscribe<mission_planner_msgs::SensorPacket>("/feedback", 1, &See0d::subDronePosition, this);
    parrotAdapter = n.subscribe<ardrone_autonomy::navdata_gps>("simulated_parrot/navdata_gps", 1, &See0d::parrotAdapterCallBack, this);
    
    map_pub = n.advertise< nav_msgs::OccupancyGrid>("/map_seed", 1,true);
    edi_pub = n.advertise<see0d::buildings>("/od_buildings_gui", 1,true);
    tree_pub = n.advertise<geometry_msgs::Polygon>("/od_tree_gui", 1,true);
    camera = n.advertise<geometry_msgs::PoseWithCovariance>("/camera", 1,true);

    bbox_service = n.serviceClient<open_data_msg::BoundingBox>("bounding_box");

  };



  ~See0d(){};

void parrotAdapterCallBack(const ardrone_autonomy::navdata_gpsConstPtr& parrotGps){

      geometry_msgs::PoseWithCovariance currentPosOfCamera;

      double latitude = (double) parrotGps->latitude;
      double longitude = (double) parrotGps->longitude;

      double cameray = distance(origin_lat,origin_lng,latitude,origin_lng, 'K')*1000 ; // in m reference to the origin
      double camerax = distance(origin_lat,origin_lng,origin_lat,longitude,'K')*1000  ;   

      currentPosOfCamera.pose.position.x = camerax - 400;
      currentPosOfCamera.pose.position.y = cameray - 400; 
     currentPosOfCamera.pose.position.z = 40; 

      currentPosOfCamera.pose.orientation.x = 70;
      // currentPosOfCamera.pose.orientation.y = 3.14;

      camera.publish(currentPosOfCamera);

    }


void subODforVisualization3D(const open_data_msg::DataConstPtr& opendata)
    {

    if(opendata->data.size() > 0) // if the response at the desidered open data exists
    {
        int ind = 0 ;

        Polygon poly;

      origin_lat = convGPS(BB_home_lat, BB_home_lng, -1*paddingy,true);
      origin_lng = convGPS(BB_home_lat, BB_home_lng, paddingx,false);
    
      String attributes_key = opendata->data[0].label.c_str();
      //ROS_INFO("labels : %s", opendata->data[0].label.c_str());
          
      printf("labels : %s \n", opendata->data[0].label.c_str());

      if(attributes_key.compare("edifici") ||  attributes_key.compare("alberate") ||  attributes_key.compare("idro")){
      for(int j = 0; j < opendata->data.size(); j++) // for on opendata
    { 


    float height = atof(opendata->data[j].attributes[0].value.c_str());
    if(height > 1.0|| attributes_key.compare("alberate") == 0 || attributes_key.compare("idro") == 0) 
    {
        geometry_msgs::Polygon messagePoly;

        for (int i=0; i < opendata->data[j].area.points.size(); i++) // number of points into the polygon
        {
          if(opendata->data[j].area.points[i].x != 0 && opendata->data[j].area.points[i].y != 0)
          { 
          double distpointy = distance(origin_lat,origin_lng,opendata->data[j].area.points[i].x,origin_lng ,'K')*1000 ; // in m reference to the origin
          double distpointx = distance(origin_lat,origin_lng,origin_lat,opendata->data[j].area.points[i].y,'K')*1000  ;   
          

          poly.push_back(Point(distpointx, distpointy));
          
          geometry_msgs::Point32 messagePoint;

          messagePoint.x = distpointx;
          messagePoint.y = distpointy;
          messagePoint.z = height;

          if(attributes_key.compare("edifici") == 0)
            messagePoly.points.push_back(messagePoint);
          
          if(attributes_key.compare("alberate") == 0)
            messageTree.points.push_back(messagePoint);


          }
          else // draw poly
          {
            if(attributes_key.compare("edifici") == 0 || attributes_key.compare("idro") == 0)
            {
          vector<Point> tmp = poly;
          const Point* elementPoints[1] = { &tmp[0] };
          int numberOfPoints = (int)tmp.size();
          fillPoly (mappa, elementPoints, &numberOfPoints, 1, 0);
          polylines(mappa, elementPoints, &numberOfPoints, 1, 0, 0, 0, 1);

          //poly_pub.publish(messagePoly);
          buildings.building.push_back(messagePoly);
          
          //fprintf(stderr,"phase 1) message polylines is %f %f %f at size %d\n",messagePoly.points[0].x,messagePoly.points[0].y,messagePoly.points[0].z,messagePoly.points.size());

             }

            if(attributes_key.compare("alberate") == 0){

              vector<Point> tmp = poly;
              const Point* elementPoints[1] = { &tmp[0] };
              circle(mappa,poly[0],4,10,-1);
            }

          if(poly.size() > 0)
          poly.clear();
          }
        }
              if(attributes_key.compare("edifici")  == 0 || attributes_key.compare("idro") == 0)
              {
              vector<Point> tmp = poly;
              const Point* elementPoints[1] = { &tmp[0] };
              int numberOfPoints = (int)tmp.size();
              fillPoly (mappa, elementPoints, &numberOfPoints, 1, 0);
              polylines(mappa, elementPoints, &numberOfPoints, 1, 0, 0, 0, 1);

              //poly_pub.publish(messagePoly);
              buildings.building.push_back(messagePoly);

              //fprintf(stderr,"phase 2) message polylines is %f %f %f at size %d\n",messagePoly.points[0].x,messagePoly.points[0].y,messagePoly.points[0].z,messagePoly.points.size());

               }

               if(attributes_key.compare("alberate") == 0){

                vector<Point> tmp = poly;
                const Point* elementPoints[1] = { &tmp[0] };
                circle(mappa,poly[0],4,0,-1);
              }      
      if(poly.size() > 0)
      poly.clear();
       
      }
      
    }

    edi_pub.publish(buildings); //pub the whole msg with building polygons

    tree_pub.publish(messageTree); //pub the whole msg with building polygons
  }
  
  // if(poly.size() != 0)
  // {
        ROS_INFO("We 1 mappa cols %d", mappa.cols);
      imwrite("/home/sgabello/catkin_ws/src/see0d/src/odomimap_edifici_totale.pgm", mappa);

  Rect roi(paddingx,paddingy,mappa.cols - paddingx*2,mappa.rows - paddingy*2);
  crop = mappa(roi).clone();
  // }
      ROS_INFO("We 2");

  if(attributes_key.compare("edifici") == 0 || attributes_key.compare("idro") == 0)
  {

  // add LTE signal
  //
  Mat map_lte = cv::Mat(Size(mappa.cols - paddingx*2,mappa.rows - paddingy*2),CV_8UC1); // same as cropped map
  map_lte.setTo(255);

  Polygon polyLte;
  int ii = 0 ;

  // read from text lte_crowd.txt
  String line;
  string temp;

  std::vector<string>  fields;
  std::vector<double>  lat_read;
  std::vector<double>  lng_read;
  std::vector<double>  lte_signal;
  std::vector<string>  typeSignal;

  double lte_threshold = -80;

  ifstream myfile ("/home/sgabello/catkin_ws/src/see0d/src/lte_crowd.txt"); // LTE in crowd sourcing- loaded so far locally. Further implemetations will access to an external database
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      
      istringstream liness( line );
      
      //ROS_INFO("that s line %s", line.c_str());

      for(int i=0; i< 11; i++){
      getline( liness, temp, ';' );

      if(i == 1) lat_read.push_back(atof(temp.c_str()));
      if(i == 2) lng_read.push_back(atof(temp.c_str()));
      if(i == 9) lte_signal.push_back(atof(temp.c_str()));
      if(i == 10) typeSignal.push_back(temp);

      }
      
      //ROS_INFO("lat %f lng %f lte %f type %s",lat_read[ii],lng_read[ii],lte_signal[ii],typeSignal[ii].c_str());

      ii++; //number of lines in the file
    }

    myfile.close();
  }

  else ROS_INFO("Unable to open file"); 

  int miao = 0 ;
  for (int i=0; i < ii ; i++)
    {
 
  double min_lat, min_lng;
  double max_lat, max_lng;

  if(BB_home_lat<BB_goal_lat)
  {
    min_lat=BB_home_lat;
    max_lat=BB_goal_lat;
  }
  else
   {
     min_lat=BB_goal_lat;
     max_lat= BB_home_lat;
   }
  if(home_lng<BB_goal_lng)
  {
    min_lng=BB_home_lng;
    max_lng=BB_goal_lng;
  }
  else
   {
    min_lng=BB_goal_lng;  
    max_lng=BB_home_lng;
    }

    if(lat_read[i] >= min_lat && lat_read[i] <= max_lat && lng_read[i] >= min_lng 
          && lng_read[i] <= max_lng && !strcmp(typeSignal[i].c_str(),"LTE") )  // gps read within the bbox and lte signal is less than threshold
          //  if(lat_read[i] >= min_lat && lat_read[i] <= max_lat && lng_read[i] >= min_lng 
      //  && lng_read[i] <= max_lng && strcmp(typeSignal[i].c_str(),"LTE")==0 )  // gps read within the bbox and lte signal present
      { 

      double y = distance(BB_home_lat,BB_home_lng,lat_read[i],BB_home_lng,'K')*1000 ; // in m reference to the origin
      double x = distance(BB_home_lat,BB_home_lng,BB_home_lat,lng_read[i],'K')*1000 ;   
                
      circle(map_lte,Point(x,y),5,20,-1); //is preferrable to pass on this area

      miao++;

      }

    }
      if(miao != 0) {

      imwrite("/home/sgabello/catkin_ws/src/see0d/src/lte_map.pgm", map_lte);
      
      bitwise_and(map_lte, crop,crop);              

      ROS_INFO("Map lte added");

      }

    if(imwrite("/home/sgabello/catkin_ws/src/see0d/src/odomimap_edifici.pgm", crop)) // save for visualization
    {
    

    ROS_INFO("Map buildings + tree created ");
    publishMap(crop.clone());

    }
  }
 }
}
   

    void publishMap(Mat m) // publish to odomi_path_planner the map
    {
      std::vector<int8_t> map;

      nav_msgs::OccupancyGrid pubMap;
      
      pubMap.header.stamp = ros::Time::now();
      pubMap.info.map_load_time = ros::Time::now();
      pubMap.header.frame_id = "map";
      pubMap.info.width = m.cols;
      pubMap.info.height = m.rows;
      pubMap.info.resolution = 1;
      pubMap.info.origin.orientation.w = 1;


      for(int j=m.rows;j > 0 ;j--)
        for(int i=0;i < m.cols;i++)
        {

          if(m.at<unsigned char>(j,i)==10)
            map.push_back(10);  
          else
          if(m.at<unsigned char>(j,i)==20)
            map.push_back(20);  
          else
            if(m.at<unsigned char>(j,i)==30)
            map.push_back(30);  
          else 
            if(m.at<unsigned char>(j,i)>100)
            map.push_back(0);
          else
            map.push_back(100);
          
        }

        pubMap.data = map;

        
        map_pub.publish(pubMap);
        
    }

    void subDronePosition(const mission_planner_msgs::SensorPacketConstPtr& spacket)
    {
      
     if(spacket->h_latit != 0 && spacket->h_longit != 0)
     {
     if(call_serviceod ) {

      ROS_INFO("Subscribing Sensor Packet drone [%f],[%f]", spacket->h_latit , spacket->h_longit);

      call_serviceod = false; //garantees only one query

      home_lat = spacket->h_latit;
      home_lng = spacket->h_longit;

      start.pose.position.x =  0; 
      start.pose.position.y =  0;  

      BB_home_lat = convGPS(home_lat, home_lng, -1*map_ext, true); //BBbox moving home
      BB_home_lng = convGPS(home_lat, home_lng, map_ext, false);

      BB_goal_lat = convGPS(home_lat, home_lng, map_ext, true);  //BBbox moving home on the diagonal called goal
      BB_goal_lng = convGPS(home_lat, home_lng, -1*map_ext, false);


      // dimensions of the whole map
      double distyBB = distance(BB_home_lat,BB_home_lng,BB_goal_lat, BB_home_lng,'K')*1000; // in m
      double distxBB = distance(BB_home_lat,BB_home_lng,BB_home_lat,BB_goal_lng,'K')*1000;


      map_width =  roundUp(distxBB, 4); // the map size should be multiple of 4 
      map_height = roundUp(distyBB, 4);

      mappa =  cv::Mat(Size(map_width + paddingy*2,map_height+ paddingy*2),CV_8UC1);
      mappa.setTo(255); // init white

      ROS_INFO("BB_goal %f,%f BB_start %f,%f ", BB_home_lat , BB_home_lng, BB_goal_lat, BB_goal_lng);

      callServiceOpendata("edifici");
      callServiceOpendata("alberate");
        }
      }
    }

    void callServiceOpendata(String lab) {   // call open data service  for the bounding box

    String whatcall = lab;

    srv.request.label = whatcall.c_str();
    srv.request.type  = 1;
    srv.request.bounding_box.points.resize(2);

    srv.request.bounding_box.points[0].x = BB_goal_lng; 
    srv.request.bounding_box.points[0].y = BB_goal_lat;
    srv.request.bounding_box.points[1].x = BB_home_lng;
    srv.request.bounding_box.points[1].y = BB_home_lat;


   // call service open data for buildings tree and rivers 
    if (bbox_service.call(srv))
    {
    if(srv.response.resp){
        ROS_INFO("Result: %s true called bbox %f %f %f %f",whatcall.c_str(), BB_goal_lat, BB_goal_lng, BB_home_lat, BB_home_lng);
        fprintf(stderr, "Result: %s true called bbox %f %f %f %f \n",whatcall.c_str(), BB_goal_lat, BB_goal_lng, BB_home_lat, BB_home_lng);
      }
    else {         
       ROS_INFO("Result: false \n");
       fprintf(stderr, "Result: false \n");
      }
    }
    else
    {
      ROS_ERROR("Failed to call service edifici bounding_box");
      
    }


    }
};



int main(int argc, char **argv)
{

  ros::init(argc, argv, "visualize_open_data");
  See0d seed;

  ros::spin();

  return 0;
}