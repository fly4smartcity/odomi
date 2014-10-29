// based on DstarDraw.cpp by James Neufeld (neufeld@cs.ualberta.ca)
// Author: Stefano Rosa
// @TODO: put everything in world coordinates, add costs to cells


#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include "mission_planner_msgs/CoordinateArray.h"
#include "mission_planner_msgs/Coordinate.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <unistd.h>
#include "Dstar.h"

#include <odomi_path_planner/mygeolib.h>

using namespace cv;
using namespace mygeolib_tool;

// parameters
int scale = 1;      // map scale (default 1 m/pixel)
int inflation=1;    // obstacles inflation radius (default 1m, should be >= GPS accuracy)
double map_ext;

Dstar *dstar;
int hh, ww;
bool ok = true;
int window;
int mbutton = 0;
int mstate = 0;
bool b_autoreplan = false;

list<waypoint> returned_wp;
ros::Publisher wp_pub;

float resolution;
int width;
int height;
geometry_msgs::Pose origin;



void main_loop()
{

  while(ros::ok())
  {
    if (b_autoreplan)
        dstar->replan();

    Mat result=dstar->draw(scale);
    if(result.cols>0 && result.rows>0)
      imshow("ODOMI",result);
    char key=cvWaitKey(10);
    switch(key) {
    case 'q':
    case 'Q':
     destroyAllWindows();
      exit(0);
      break;
    case 'r':
    case 'R':
      dstar->replan();
      break;
    case 'a':
    case 'A':
      b_autoreplan = !b_autoreplan;
      break;
    case 'c':
    case 'C':
      dstar->init(40,50,140, 90);
      break;
    }
    ros::spinOnce();
  }
}



void mouseFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          dstar->updateStart(x/scale, y/scale);
           dstar->replan();
        //dstar->getWaypoints();
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          dstar->updateGoal(x/scale, y/scale);
           dstar->replan();
        //dstar->getWaypoints();
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if ( event == EVENT_MOUSEMOVE )
     {

     }
}



void receiveMap(const boost::shared_ptr<const nav_msgs::OccupancyGrid> map)
{
  ROS_INFO("odomi_path_planner: received map");

  nav_msgs::MapMetaData info=map->info;

  resolution= info.resolution;
  width = info.width;
  height = info.height;
  origin= info.origin;

  Mat obstacles(height,width,CV_8UC1);

  obstacles.setTo(255);
  dstar->init(0,0,width,height);

  for(int i=0;i<width;i++)
    for(int j=0;j<height;j++)
    {
      int8_t cell=map->data[i+j*width];

        if(cell>0 && cell<127)
        obstacles.at<unsigned char>(height-1-j,i)=0; // obstacle
        else
         if(cell>=127)
        obstacles.at<unsigned char>(height-1-j,i)=255; // free cell


      if(cell == 10)
          obstacles.at<unsigned char>(height-1-j,i)=10; // trees

      if(cell == 20)
          obstacles.at<unsigned char>(height-1-j,i)=20; // no LTE cell
      
      if(cell == 30)
        obstacles.at<unsigned char>(height-1-j,i)=30; // river cell

  
      
    }

  resize(obstacles,obstacles,Size(obstacles.cols/scale,obstacles.rows/scale));

  rectangle(obstacles,Point(0,0),Point(obstacles.cols-1,obstacles.rows-1),0,1);
  circle(obstacles,Point(map_ext,map_ext),8,255,-1);
  circle(obstacles,Point(width-1-map_ext,height-1-map_ext),8,255,-1);



  for(int i=0;i<obstacles.cols;i++)
    for(int j=0;j<obstacles.rows;j++)
    {
      unsigned char cost= obstacles.at<unsigned char>(j,i);
      if(cost == 50)
        dstar->updateCell(i,j,1); // rivers
    
      if(cost == 20)
      {
        //printf(".");
        dstar->updateCell(i,j,1); // LTE signal
      }


    }
printf("\n");



  //threshold( obstacles, obstacles, 100, 255,0 );
  //imshow("thresolded",obstacles);

  int morph_elem = MORPH_ELLIPSE;
  int morph_size = inflation;
  Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
  morphologyEx( obstacles, obstacles, 0, element );
  dstar->setObstacles(obstacles);

  for(int i=0;i<obstacles.cols;i++)
    for(int j=0;j<obstacles.rows;j++)
    {
      unsigned char cost= obstacles.at<unsigned char>(j,i);


      if(cost==0)
        dstar->updateCell(i,j, -1);  //obstacles if 1 is preferred zone if -1 is obstalce
      if(cost==10)
        dstar->updateCell(i,j, -1);  //tree


    }

  mission_planner_msgs::CoordinateArray wp;
  mission_planner_msgs::Coordinate      wp_coordinate;
  
  dstar->updateStart(map_ext,map_ext);
  dstar->updateGoal((width-map_ext)/scale,(height-map_ext)/scale);

  dstar->replan();

  // list<waypoints>

  returned_wp = dstar->getWaypoints();
  // ROS_INFO ("returned_wp %d\n ",returned_wp.size());

  //Is it the right path ?
  if(returned_wp.size() < 3)
  {

   ROS_WARN ("odomi_path_planner: planning failed, replanning");

   //position.x =
   //position.y = position.y +10;


  }




  std::list<waypoint>::const_iterator iterator;

  for(iterator = returned_wp.begin(); iterator != returned_wp.end(); ++iterator)
  {
  wp_coordinate.latitude = (*iterator).y;
  wp_coordinate.longitude = (*iterator).x;

  //printf("wp lat %f wp lng %f\n", wp_coordinate.latitude, wp_coordinate.longitude);

  wp.waypoint.push_back(wp_coordinate);
  }



  wp_pub.publish(wp);

}





void receiveGoal(const boost::shared_ptr<const geometry_msgs::PoseStamped> goal)
{

  mission_planner_msgs::CoordinateArray wp;
  mission_planner_msgs::Coordinate      wp_coordinate;
  geometry_msgs::Point position = goal->pose.position;
  geometry_msgs::Quaternion orientation = goal->pose.orientation;
  ROS_INFO("received goal (position): %f %f %f" ,position.x,position.y,position.z);
  ROS_INFO("received goal (orientation quaternion): %f %f %f %f",orientation.x ,orientation.y ,orientation.z ,orientation.w);
  //cout << "convert to angle" << 2 * acos(orientation.w) << endl;
  /*tf::TransformListener listener;

  tf::StampedTransform transform;
  try{
   listener.lookupTransform("/map", "/base_link",
                            ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
  ROS_ERROR("Robot pose not available: %s",ex.what());
  }*/


  //dstar->updateStart(100,100);
  //dstar->updateGoal((position.x - origin.position.x)/scale/resolution, height/scale-(position.y - origin.position.y)/scale/resolution);


  while(ok){

  dstar->updateGoal(position.x/scale/resolution,position.y/scale/resolution);
  dstar->replan();

  // list<waypoints>

  returned_wp = dstar->getWaypoints();
  // ROS_INFO ("returned_wp %d\n ",returned_wp.size());

  //Is it the right path ?
  if(returned_wp.size() < 3)
  {

   ROS_WARN ("odomi_path_planner: planning failed, replanning");

   //position.x =
   position.y = position.y +10;


  } else ok = false;

  }

 /*

  dstar->updateGoal(position.x/scale/resolution,position.y/scale/resolution);
  dstar->replan();

  // list<waypoints>

  returned_wp = dstar->getWaypoints();

*/


  std::list<waypoint>::const_iterator iterator;

  for(iterator = returned_wp.begin(); iterator != returned_wp.end(); ++iterator)
  {
  wp_coordinate.latitude = (*iterator).y;
  wp_coordinate.longitude = (*iterator).x;

  ROS_INFO("waypoint lat %f wp lng %f\n", wp_coordinate.latitude, wp_coordinate.longitude);

  wp.waypoint.push_back(wp_coordinate);
  }



  //wp_pub.publish(wp);


  }


void receiveStart(const boost::shared_ptr<const geometry_msgs::PoseStamped> start)
{
  geometry_msgs::Point position = start->pose.position;
  geometry_msgs::Quaternion orientation = start->pose.orientation;
  cout << "###received start (position): " << position.x << ' ' << position.y << ' ' << position.z << endl;
  cout << "###received start (orientation): " << orientation.x << ' ' << orientation.y << ' ' << orientation.z << ' ' << orientation.w <<endl;
  //cout << "convert to angle" << 2 * acos(orientation.w) << endl;
  /*tf::TransformListener listener;

  tf::StampedTransform transform;
  try{
   listener.lookupTransform("/map", "/base_link",
                            ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
  ROS_ERROR("Robot pose not available: %s",ex.what());
  }*/


  //dstar->updateStart(100,100);
//  dstar->updateStart((position.x - origin.position.x)/scale/resolution, height/scale-(position.y - origin.position.y)/scale/resolution);
  dstar->updateStart(position.x/scale/resolution,position.y/scale/resolution);
  dstar->replan();
  //dstar->getWaypoints();



}



int main(int argc, char **argv) {

  // init ros
  ros::init(argc, argv, "odomi_path_planner");
  ros::NodeHandle nh;
  ros::Subscriber lte_sub;
  ros::NodeHandle private_nh("~");
  private_nh.param("bounding_box_extension", map_ext, 200.00);



 namedWindow("ODOMI", 1);

setMouseCallback("ODOMI", mouseFunc, NULL);

  dstar = new Dstar();
  dstar->init(30,50,100, 90);

  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1,  receiveGoal);
  ros::Subscriber start_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/start", 1,  receiveStart);
  ros::Subscriber obstacles_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 2,  receiveMap); //mission map

  // publish waypoints
  wp_pub = nh.advertise<mission_planner_msgs::CoordinateArray>("/2mp_wp", 2,true);

 main_loop();
   //main_op();
  return 1;
}
