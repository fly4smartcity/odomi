// based on DstarDraw.cpp by James Neufeld (neufeld@cs.ualberta.ca)
// Author: Stefano Rosa
// @TODO: dimensions of the map, get map from ros, put everything in world coordinates,
//        add costs to cells, publish waypoints  
// 
//


#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"
#include <unistd.h>
#include "Dstar.h"
#include "mission_planner_msgs/CoordinateArray.h"
#include "mission_planner_msgs/Coordinate.h"

#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>


using namespace cv;

int hh, ww;
bool ok = true;


int window; 
Dstar *dstar;

int scale = 1;
int mbutton = 0;
int mstate = 0;

bool b_autoreplan = false;

int inflation=1;

// things to pub wp

list<waypoint> returned_wp;
ros::Publisher wp_pub;

// map info
  float resolution;
  int width;
  int height;
  geometry_msgs::Pose origin;



void main_loop()
{

  while(ros::ok())
  {
    if (b_autoreplan) 
      {
        dstar->replan();
        //dstar->getWaypoints();
      }

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

void main_op()
{
while(ros::ok())
  {
    if (b_autoreplan) 
      {
        dstar->replan();
        //dstar->getWaypoints();
      }

    Mat result=dstar->draw(scale);
    if(result.cols>0 && result.rows>0)
      imshow("ODOMI",result);
    /*char key=cvWaitKey(10);
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
    }*/
dstar->init(0,0,140, 90);
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



void loadBuildings(string filename)
{
  Mat buildings=imread(filename,0);  
  resize(buildings,buildings,Size(800/scale,600/scale));
  threshold( buildings, buildings, 100, 255,0 );

  int morph_elem = MORPH_ELLIPSE;
int morph_size = inflation;
  Mat element = getStructuringElement( morph_elem, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    morphologyEx( buildings, buildings, 0, element );
 
  dstar->setObstacles(buildings);

}

void loadLte(string filename)
{
  Mat lte=imread(filename,0);  
  resize(lte,lte,Size(800/scale,600/scale));
//imshow("build",buildings); cvWaitKey(0);
  for(int i=0;i<lte.cols;i++)
      for(int j=0;j<lte.rows;j++)
      {
        int x=i; int y=j;
  // y = hh -y+scale/2;
  // x += scale/2;
  
  // y /= scale;
  // x /= scale;
  
  unsigned char cost= lte.at<unsigned char>(j,i);  
  if(cost < 200){
      dstar->updateCell(x, y, cost/100.0);
      printf("cost %f\n",cost/100.0);
  }
  }
}

void loadLteObstacles(string filename)
{
  Mat lte=imread(filename,0);  
  resize(lte,lte,Size(800/scale,600/scale));
//imshow("build",buildings); cvWaitKey(0);
  for(int i=0;i<lte.cols;i++)
      for(int j=0;j<lte.rows;j++)
      {
        int x=i; int y=j;
  // y = hh -y+scale/2;
  // x += scale/2;
  
  // y /= scale;
  // x /= scale;
  
  unsigned char cost= lte.at<unsigned char>(j,i);  
  if(cost < 10){
      dstar->updateCell(x, y, -2);
      
  }
  }
}


void receiveMap(const boost::shared_ptr<const nav_msgs::OccupancyGrid> map)
{
  cout << "###received map\n";
  nav_msgs::MapMetaData info=map->info;

  resolution= info.resolution;
  width = info.width;
  height = info.height;
  origin= info.origin;


  Mat obstacles(height,width,CV_8UC1);
  obstacles=Scalar(127);
  for(int i=0;i<width;i++)
    for(int j=0;j<height;j++)
    {
      int8_t cell=map->data[i+j*width];
      if(cell>0 && cell<127)
        obstacles.at<unsigned char>(height-1-j,i)=0;
      else
         if(cell>=127)
        obstacles.at<unsigned char>(height-1-j,i)=255;
    }

  resize(obstacles,obstacles,Size(obstacles.cols/scale,obstacles.rows/scale));
  //imshow("obstacles",obstacles);
  
  rectangle(obstacles,Point(0,0),Point(obstacles.cols-1,obstacles.rows-1),0,1);	
  circle(obstacles,Point(0,0),4,255,-1);    
  circle(obstacles,Point(width-1,height-1),4,255,-1);  

  threshold( obstacles, obstacles, 100, 255,0 );
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
      if(cost<10)
        dstar->updateCell(i,j, -1);
    }  


}


void receiveGoal(const boost::shared_ptr<const geometry_msgs::PoseStamped> goal)
{

  mission_planner_msgs::CoordinateArray wp;
  mission_planner_msgs::Coordinate      wp_coordinate;
  geometry_msgs::Point position = goal->pose.position;
  geometry_msgs::Quaternion orientation = goal->pose.orientation;
  cout << "###received goal (position): " << position.x << ' ' << position.y << ' ' << position.z << endl;
  cout << "###received goal (orientation): " << orientation.x << ' ' << orientation.y << ' ' << orientation.z << ' ' << orientation.w <<endl;
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
  ROS_INFO ("returned_wp %d\n ",returned_wp.size());

  //Is it the right path ? 
  if(returned_wp.size() < 3) 
  {
   
   ROS_INFO ("---Goal not good replanning -----\n ");

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

  printf("wp lat %f wp lng %f\n", wp_coordinate.latitude, wp_coordinate.longitude);

  wp.waypoint.push_back(wp_coordinate);
  }
  


  wp_pub.publish(wp);


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

  


 namedWindow("ODOMI", 1);

setMouseCallback("ODOMI", mouseFunc, NULL);
//createTrackbar("inflation", "ODOMI", &inflation, 9);
  // // init glut
  // glutInit(&argc, argv);  
  // glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);  
  // glutInitWindowSize(800, 600);  
  // glutInitWindowPosition(20, 20);  
    
  // window = glutCreateWindow("Dstar Visualizer");  
  
  // glutDisplayFunc(&DrawGLScene);  
  // glutIdleFunc(&DrawGLScene);
  // glutReshapeFunc(&ReSizeGLScene);
  // glutKeyboardFunc(&keyPressed);
  // glutMouseFunc(&mouseFunc);
  // glutMotionFunc(&mouseMotionFunc);

  // InitGL(800, 600);


  

  dstar = new Dstar();
  dstar->init(30,50,100, 90);
  
  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1,  receiveGoal);
  ros::Subscriber start_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/start", 1,  receiveStart);
  ros::Subscriber obstacles_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 2,  receiveMap); //mission map

  // publish waypoints
  wp_pub = nh.advertise<mission_planner_msgs::CoordinateArray>("/2mp_wp", 2,true);
												//dynamic map (sum of the two)

  //loadLteObstacles("lte.jpg");
  //loadBuildings("/home/stefano/ros_workspace/odomi_path_planner/salaD1.pgm");
/*
  printf("----------------------------------\n");
  printf("ODOMI Path Planner\n");
  printf("Commands:\n");
  printf("[q/Q] - Quit\n");
  printf("[r/R] - Replan\n");
  printf("[a/A] - Toggle Auto Replan\n");
  printf("[c/C] - Clear (restart)\n");
  printf("left mouse click - make cell untraversable (cost -1)\n");
  printf("middle mouse click - move goal to cell\n");
  printf("right mouse click - move start to cell\n");
  printf("----------------------------------\n");*/

  //glutMainLoop();  

 main_loop();
   //main_op();
  return 1;
}
