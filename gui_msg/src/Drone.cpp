/* reference
// http://answers.ros.org/question/10905/can-a-node-act-as-a-server-and-a-publisher/
*/
#include "ros/ros.h"
#include "gui_msg/Drone.h"
#include "gui_msg/SensorPacket.h"

#include <stdlib.h> // abs()
#include <cmath>

ros::NodeHandle* n_pub;
ros::NodeHandle* n_sub;
ros::Publisher ROSGUI_drone;








////////////////////////////mission->gps_waypoint[i].latitude
///////////////////////////////////////////////////////////////////////////

  void interpola(float GpsLatA,float GpsLngA,float GpsLatB,float GpsLngB,const gui_msg::Drone::ConstPtr& mission){


//var di interpolazione per il numero di punti
  int k=215;//num di punti path

  int n=10;//per randnum
  int randnum;
  /***************
  GENERO NUMERO CASUALE X MESSAGGI
  *****************/
  randnum=(int)((2*(rand()%n))-(n));//generate rnd number tra n e -n
  //coordinate GPS home
  /*float GpsLatHome=(float)mission->home.latitude;
  float GpsLngHome=(float)mission->home.longitude;
  float GpsLatGoal=(float)mission->target_position.latitude;
  float GpsLngGoal=(float)mission->target_position.longitude;*/
  
  //frequenza di lavoro
  ros::Rate loop_rate(65);
  gui_msg::SensorPacket msg;//istanzio il messaggio del publisher
  ROS_INFO("Il drone è partito");
  int i=0;
  while (ros::ok() && i<=k)
  {
    //simulo il percorso    
    //gps corrente
    msg.c_latit=GpsLatA+(((GpsLatB-GpsLatA)/(k))*(k-(k-i)));
    msg.c_longit=GpsLngA+(((GpsLngB-GpsLngA)/(k))*(k-(k-i)));
    msg.c_altit=(randnum*100)-(i*10);//int32 altitudine
    //gps home
    msg.h_latit=GpsLatA;//float64
    msg.h_longit=GpsLngA;//float64
    msg.h_altit=0;//int32
    //gps target
    //msg.t_latit=GpsLatB;
    //msg.t_longit=GpsLngB;//float64
    //msg.t_altit=0;//int32
    //
    msg.current_wayp=0;//uint8
    msg.tot_wayp=mission->gps_waypoint.size();//uint8
    msg.altimeter=msg.c_altit;//int16 altimetro
    msg.fly_time=0;//uint16
    msg.grd_speed=(130*(double)i/k);//uint8
    msg.top_speed=0;//int16
    msg.compass_heading=abs(randnum)-(i);//int16 bussola
    msg.roll_ang=45*sin(i*0.1);//(2*(rand()%n))-(n);//int8
    msg.pitch_ang=20*sin(i*0.1);//(2*(rand()%n))-(n);//int8
    msg.battery=(int)(100-(100*(double)i/k));//uint8 batteria
//ROS_INFO("battary:%f\n\ti:%d\n\tk:%d\n\ti/k:%f",100-(100*(double)i/k),i,k,(double)i/k);

    //pubblico il mex
    ROSGUI_drone.publish(msg);
    
   // ROS_INFO("Mesasggio inviato %2d:\n\tlatitude: %f\n\tlongitude: %f\n\tbattery: %d",i,msg.c_latit,msg.c_longit,msg.battery);

      //STAMPO quello inviato    
      //ROS_INFO("\nGPSLATHOME:\t%f\nGPSLNGHOME:\t%f\nGPSLATP1:\t%f\nGPSLNGP1:\t%f\nGPSLATP2:\t%f\nGPSLNGP2:\t%f\nGPSLATP3:\t%f\nGPSLNGP3:\t%f\nGPSLATP4:\t%f\nGPSLNGP4:\t%f\nGPSLATP5:\t%f\nGPSLNGP5:\t%f\nGPSLATPGOAL:\t%f\nGPSLNGPGOAL:\t%f",msg.GPSLATHOME,msg.GPSLNGHOME,msg.GPSLATP1,msg.GPSLNGP1,msg.GPSLATP2,msg.GPSLNGP2,msg.GPSLATP3,msg.GPSLNGP3,msg.GPSLATP4, msg.GPSLNGP4,msg.GPSLATP5,msg.GPSLNGP5,msg.GPSLATGOAL,msg.GPSLNGGOAL);
    ros::spinOnce();
    loop_rate.sleep();
    i++;
  }
  ROS_INFO("Il drone è arrivato");  



}//interpola
//////////////////////////////////////////////////////////////////////////////////////////////////////














void catchMission(const gui_msg::Drone::ConstPtr& mission)
{
  


if(mission->gps_waypoint.size() != 0 ){

interpola(mission->home.latitude,mission->home.longitude,mission->gps_waypoint[0].latitude,mission->gps_waypoint[0].longitude,mission);

int k;
for(k=1;k < mission->gps_waypoint.size();k++)
interpola(mission->gps_waypoint[k-1].latitude,mission->gps_waypoint[k-1].longitude,mission->gps_waypoint[k].latitude,mission->gps_waypoint[k].longitude,mission);

ROS_INFO("Calcolo path con starting point gps.lat %f, gps.long  %f mis.lat %f, mis.long  %f \n", mission->gps_waypoint[k].latitude, mission->gps_waypoint[k].longitude,mission->target_position.latitude,mission->target_position.longitude);

interpola(mission->gps_waypoint[k-1].latitude,mission->gps_waypoint[k-1].longitude,mission->target_position.latitude,mission->target_position.longitude,mission);

}
else
{

interpola(mission->home.latitude,mission->home.longitude,mission->target_position.latitude,mission->target_position.longitude,mission);

}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Drone");
  n_pub=new ros::NodeHandle;
  n_sub=new ros::NodeHandle;
  ros::Subscriber sub = n_sub->subscribe("drone", 1000, catchMission);
  //ros::ServiceServer service = n_srv->advertiseService("target", catchTarget);
  ROSGUI_drone = n_pub->advertise<gui_msg::SensorPacket>("feedback", 1000);
  ROS_INFO("Ready to catch a mission.");
  ros::spin();
  return 0;
}
