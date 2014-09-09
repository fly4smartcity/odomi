/* referenze
// http://answers.ros.org/question/10905/can-a-node-act-as-a-server-and-a-publisher/
// http://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/
*/

#include "ros/ros.h"
#include "gui_msg/getCurrentPosition.h"
#include "gui_msg/Drone.h"

//istanzio i nodehandle
ros::NodeHandle* n_pub;
ros::NodeHandle* n_srv;
//istanzio il publisher
ros::Publisher ROSGUI_drone;
//istanzio il messaggio del publisher
gui_msg::Drone msg;
bool flag_srv=false;

bool catchTarget(gui_msg::getCurrentPosition::Request  &req, gui_msg::getCurrentPosition::Response &res)
{
  //variabile di interpolazione per il numero di punti
  int k=5;
  //coordinate GPS home e del target
  float GpsLatHome=45.05;
  float GpsLngHome=7.64;

  // Goal coordinates
 // float GpsLatGoal=(float)req.target_position[0];
 // float GpsLngGoal=(float)req.target_position[0];

 float GpsLatGoal= (float)req.latitude; 
 float GpsLngGoal= (float)req.longitude;

  //stampo su shell il valore target letto
  ROS_INFO("request: latitude=%f, longitude=%f", GpsLatGoal, GpsLngGoal);
  if (true)
  {
    //Publisher path
    res.res=true;   
    //frequenza di lavoro [Hz]
    ros::Rate loop_rate(1);
    //pulisco il vettore di waypoint contenuto nel messaggio
    msg.gps_waypoint.clear();    
    //riempo il messaggio
    //HOME
    msg.home.latitude=GpsLatHome;
    msg.home.longitude=GpsLngHome;
    //TARGET
    msg.target_position.latitude=GpsLatGoal;
    msg.target_position.longitude=GpsLngGoal;
    //PATH
    int i=0;
    int c;
    while (ros::ok() && i<=0)//ne invio 1
    {
      ROS_INFO("Inizio a riempire il vettore");
      //CREO IL PATH
      for(c=0;c<=k;++c)
      {
        gui_msg::Coordinate point;//istanzio il punto
        point.latitude=GpsLatHome+(((GpsLatGoal-GpsLatHome)/(k))*(k-(k-c)));
        point.longitude=GpsLngHome+(((GpsLngGoal-GpsLngHome)/(k))*(k-(k-c)));
        point.altitude=0;
        //lo inserisco nel vettore
        msg.gps_waypoint.push_back(point);
        ROS_INFO("Posizione %2d:\n\tlatitude: %f\n\tlongitude: %f",c,point.latitude,point.longitude);
      }
      
      ros::spinOnce();
      loop_rate.sleep();
      i++;
    }//PATH
  }
  else
    res.res=false;
  ROS_INFO("sending back response: [%d]", res.res);
  flag_srv=true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Mission_Planner");
  //inizializzo i nodehandle
  n_pub=new ros::NodeHandle;
  n_srv=new ros::NodeHandle;
 
  ros::ServiceServer service = n_srv->advertiseService("target", catchTarget);
 
  ROSGUI_drone = n_pub->advertise<gui_msg::Drone>("drone", 1000);
  ROS_INFO("Ready to catch targets.");

  //definisco la frequenza di lavoro del main
  ros::Rate loop_ratemain(10);
  while(ros::ok())
  {
    if(flag_srv)
    {
      //pubblico il path di punti sul topic
      ROSGUI_drone.publish(msg);
      flag_srv=false;
    }
    ros::spinOnce();
    loop_ratemain.sleep();
  }
  return 0;
}
