#include "ros/ros.h"
#include <mission_planner_msgs/Drone.h>
#include <mission_planner_msgs/getCurrentPosition.h>
#include <mission_planner_msgs/telemetry.h>
#include <mission_planner_msgs/SensorPacket.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/navdata_gps.h>

#include <ctime>
#include <stdlib.h> 
#include <cmath>

using namespace std;

class Drone
{

  private:

   ros::NodeHandle n;
   ros::Publisher n_pub, navdata_gps, navdata;
   ros::Subscriber n_sub;
   ros::NodeHandle private_nh;

    // for telemetry activation
    ros::ServiceServer telemetryActivation;
    bool first;
    mission_planner_msgs::SensorPacket msg;

  public:

	double lat,lng,alt;

    Drone()
    {
    
      ros::NodeHandle private_nh("~");
      private_nh.param("latitude", lat, 45.062075); // home coordinates
      private_nh.param("longitude",lng, 7.663291);
      private_nh.param("altitude",alt, 0.00); //starting from the ground

      

      n_sub = n.subscribe("drone", 1000, &Drone::catchMission, this);

      //service telemetry_activation
      telemetryActivation = n.advertiseService("telemetry_activation", &Drone::telemtryActivation , this);

	  //publisher feedback
  	  n_pub = n.advertise<mission_planner_msgs::SensorPacket>("feedback", 1000);
  	  navdata_gps = n.advertise<ardrone_autonomy::navdata_gps>("simulated_parrot/navdata_gps", 1000);
  	  navdata = n.advertise<ardrone_autonomy::Navdata>("simulated_parrot/Navdata", 1000);

	}


    ~Drone(){
    }

	void catchMission(const mission_planner_msgs::Drone::ConstPtr& mission)
	{

      ROS_INFO("Travelling without moving... I am really not moving yeah (simulation only)");
	    //var di interpolazione per il numero di punti
	  int k=10;//num di punti path

	  int n=10;//per randnum
	  int randnum ;
	  randnum=1;//(int)((2*(rand()%n))-(n));//generate rnd number tra n e -n

	  for (int i = 1 ; i < mission->movements.size()-7; i++){
	  //coordinate GPS home
	  float GpsLatHome,GpsLngHome, GpsLatGoal, GpsLngGoal;	
	  if ( i == 1){
	  GpsLatHome=(float)mission->home.latitude;
	  GpsLngHome=(float)mission->home.longitude;
	  GpsLatGoal=(float)mission->movements[i].target_position.latitude;
	  GpsLngGoal=(float)mission->movements[i].target_position.longitude;
	  }
	  else{

	  GpsLatHome=(float)mission->movements[i-1].target_position.latitude;
	  GpsLngHome=(float)mission->movements[i-1].target_position.longitude;
	  GpsLatGoal=(float)mission->movements[i].target_position.latitude;
	  GpsLngGoal=(float)mission->movements[i].target_position.longitude;

	  }	  //frequenza di lavoro
	  ros::Rate loop_rate(65);
	  mission_planner_msgs::SensorPacket msg;
	  ardrone_autonomy::navdata_gps gps;

	  ROS_INFO("Il drone è partito");
	  int i=0;
	  while (ros::ok() && i<=k)
	  {
	    //simulo il percorso    
	    //gps corrente
	    msg.c_latit=GpsLatHome+(((GpsLatGoal-GpsLatHome)/(k))*(k-(k-i)));
	    msg.c_longit=GpsLngHome+(((GpsLngGoal-GpsLngHome)/(k))*(k-(k-i)));
	    msg.c_altit= 10;//int32 altitudine

	    gps.latitude = msg.c_latit;
	    gps.longitude = msg.c_longit;

	    //gps home
	    msg.h_latit=GpsLatHome;//float64
	    msg.h_longit=GpsLngHome;//float64
	    msg.h_altit=0;//int32
	    //gps target
	    msg.t_latit=GpsLatGoal;
	    msg.t_longit=GpsLngGoal;//float64
	    msg.t_altit=0;//int32
	    //
	    msg.current_wayp=0;//uint8
	    //msg.tot_wayp=mission->gps_waypoint.size();//uint8
	    msg.altimeter=msg.c_altit;//int16 altimetro
	    msg.fly_time=0;//uint16
	    msg.grd_speed=(130*(double)i/k);//uint8
	    msg.top_speed=0;//int16
	    msg.compass_heading=abs(randnum)-(i);//int16 bussola
	    msg.roll_ang=45*sin(i*0.1);//(2*(rand()%n))-(n);//int8
	    msg.pitch_ang=20*sin(i*0.1);//(2*(rand()%n))-(n);//int8
	    msg.battery=(int)(100-(100*(double)i/k));//uint8 batteria
	//ROS_INFO("battary:%f\n\ti:%d\n\tk:%d\n\ti/k:%f",100-(100*(double)i/k),i,k,(double)i/k);
	    
	    ROS_INFO("Mesasggio inviato %2d:\n\tlatitude: %f\n\tlongitude: %f\n\tbattery: %d",i,msg.c_latit,msg.c_longit,msg.battery);

	    n_pub.publish(msg);
	    navdata_gps.publish(gps);
	      
	    //ROS_INFO("\nGPSLATHOME:\t%f\nGPSLNGHOME:\t%f\nGPSLATP1:\t%f\nGPSLNGP1:\t%f\nGPSLATP2:\t%f\nGPSLNGP2:\t%f\nGPSLATP3:\t%f\nGPSLNGP3:\t%f\nGPSLATP4:\t%f\nGPSLNGP4:\t%f\nGPSLATP5:\t%f\nGPSLNGP5:\t%f\nGPSLATPGOAL:\t%f\nGPSLNGPGOAL:\t%f",msg.GPSLATHOME,msg.GPSLNGHOME,msg.GPSLATP1,msg.GPSLNGP1,msg.GPSLATP2,msg.GPSLNGP2,msg.GPSLATP3,msg.GPSLNGP3,msg.GPSLATP4, msg.GPSLNGP4,msg.GPSLATP5,msg.GPSLNGP5,msg.GPSLATGOAL,msg.GPSLNGGOAL);
	    ros::spinOnce();
	    loop_rate.sleep();
	    i++;	

	}
	}
 		ROS_INFO("Drone is arrived !");  

   }
	bool telemtryActivation(mission_planner_msgs::telemetry::Request  &req, mission_planner_msgs::telemetry::Response &res)
	{

	if(req.activate) 
	{

	ROS_INFO("Sending telemetry");
	res.resp = true;

	first = true;
	if(first)
	{
	   
	    msg.c_latit=lat;//float64
   	    msg.c_longit=lng;//float64
	    msg.c_altit=alt;//int32 altitudine

	    //gps home
	    msg.h_latit=msg.c_latit;//float64  
	    msg.h_longit=msg.c_longit;//float64
	    msg.h_altit=0;//int32

	    msg.current_wayp=0;//uint8

	    msg.altimeter=msg.c_altit;//int16 
	    msg.fly_time=0;//uint16
	    msg.grd_speed=0;//uint8
	    msg.top_speed=0;//int16
	    msg.compass_heading=10;//int16 
	    msg.roll_ang=0;//(2*(rand()%n))-(n);//int8
	    msg.pitch_ang=0;//(2*(rand()%n))-(n);//int8
	    msg.battery=168;//uint8 

	    //pubblico il mex
	    n_pub.publish(msg);
	    first = false;
	 }



	return true;

	}else

	{
	ROS_INFO("Having some problems...");
	res.resp = false;
	return false;
	}
	}

	};


	


	int main(int argc, char **argv)
	{
  		ros::init(argc, argv, "Drone");
  		Drone dr;
  		ROS_INFO("Ready to receive a mission.");
  		ros::spin();

  		return 0;
	}
