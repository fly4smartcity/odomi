/* reference
// http://answers.ros.org/question/10905/can-a-node-act-as-a-server-and-a-publisher/
*/
#include "ros/ros.h"
#include <mission_planner_msgs/Drone.h>
#include <mission_planner_msgs/getCurrentPosition.h>
#include <mission_planner_msgs/telemetry.h>
#include <mission_planner_msgs/SensorPacket.h>
#include <ctime>
/*#include "gui_msg/Drone.h"
#include "gui_msg/SensorPacket.h"
*/
#include <stdlib.h> // abs()
#include <cmath>


using namespace std;

ros::NodeHandle* n_pub;
ros::NodeHandle* n_sub;
ros::Publisher ROSGUI_drone;

// for telemetry activation
ros::ServiceServer telemetryActivation;

mission_planner_msgs::SensorPacket msg;//istanzio il messaggio del publisher

bool first;



////////////////////////////mission->gps_waypoint[i].latitude
///////////////////////////////////////////////////////////////////////////

  void interpola(float GpsLatA,float GpsLngA,float GpsLatB,float GpsLngB, const mission_planner_msgs::Drone::ConstPtr& mission){


  //var di interpolazione per il numero di punti
  int k=20;//num di punti path

  int n=10;//per randnum
  int randnum;
  randnum=(int)((2*(rand()%n))-(n));//generate rnd number tra n e -n
  
  //frequenza di lavoro
  ros::Rate loop_rate(65);
  
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
    /*
    msg.h_latit=45.079006;//float64
    msg.h_longit=7.61466;//float64
    msg.h_altit=0;//int32 
    */
   
    msg.current_wayp=0;//uint8
    //msg.tot_wayp=mission->gps_waypoint.size();//uint8
    msg.altimeter=msg.c_altit;//int16 altimetro
    msg.fly_time=0;//uint16
    msg.grd_speed=(130*(double)i/k);//uint8
    msg.top_speed=0;//int16
    msg.compass_heading=abs(randnum)-(i);//int16 bussola
    msg.roll_ang=45*sin(i*0.1);//(2*(rand()%n))-(n);//int8
    msg.pitch_ang=20*sin(i*0.1);//(2*(rand()%n))-(n);//int8
    msg.battery=(int)(168-(138*(double)i/k));//uint8 batteria

    //pubblico il mex
    ROSGUI_drone.publish(msg);
    
    ros::spinOnce();
    loop_rate.sleep();
    i++;
  }
  ROS_INFO("Il drone è arrivato");  



}//interpola
//////////////////////////////////////////////////////////////////////////////////////////////////////



	void catchMission(const mission_planner_msgs::Drone::ConstPtr& mission)
	{
  
	for(int i=0; i <= mission->movements.size(); i++){

	if(mission->movements[i].type == 4 || mission->movements[i].type == 6 ){ // go to on circle or circling with a set radius
	
	ROS_INFO("Travelling without moving... ;) \n");
	interpola(msg.c_latit,msg.c_longit,mission->movements[i].target_position.latitude,mission->movements[i].target_position.longitude, mission);
	}
	
	if( mission->movements[i].type == 5){//Hover
	}


	}

	}

	bool telemtryActivation(mission_planner_msgs::telemetry::Request  &req, mission_planner_msgs::telemetry::Response &res)
	{
		
	if(req.activate) // if it s true	
	{
	
	ROS_INFO("Sending telemetry \n");
	res.resp = true;	
	
	first = true;
	if(first)
	{
	    //simulo il percorso    
	    //gps corrente
	    msg.c_latit=45.062238;//float64
   	    msg.c_longit=7.663083;//float64
	    msg.c_altit=15;//int32 altitudine

	    //gps home
	    msg.h_latit=45.062238;//float64
	    msg.h_longit=7.663083;//float64
	    msg.h_altit=0;//int32
   
	    msg.current_wayp=0;//uint8
	    
	    msg.altimeter=msg.c_altit;//int16 altimetro
	    msg.fly_time=0;//uint16
	    msg.grd_speed=0;//uint8
	    msg.top_speed=0;//int16
	    msg.compass_heading=10;//int16 bussola
	    msg.roll_ang=0;//(2*(rand()%n))-(n);//int8
	    msg.pitch_ang=0;//(2*(rand()%n))-(n);//int8
	    msg.battery=168;//uint8 batteria

	    //pubblico il mex
	    ROSGUI_drone.publish(msg);
	    first = false;
	 }   
	   
  	

	return true;

	}else

	{
	ROS_INFO("Getting some problems...\n");
	res.resp = false;	
	return false;
	}
	
	}	
	

	int main(int argc, char **argv)
	{
  		ros::init(argc, argv, "Drone");
  		n_pub=new ros::NodeHandle;
  		n_sub=new ros::NodeHandle;
		ros::NodeHandle n;

  		ros::Subscriber sub = n_sub->subscribe("drone", 1000, catchMission);

		//service telemetry_activation
		telemetryActivation = n.advertiseService("telemetry_activation",telemtryActivation);

		//publisher feedback
  		ROSGUI_drone = n_pub->advertise<mission_planner_msgs::SensorPacket>("feedback", 1000);

  		ROS_INFO("Ready to catch a mission.");
  		ros::spin();
  		return 0;
	}
