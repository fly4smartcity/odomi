#include "ros/ros.h"
#include <mission_planner_msgs/Drone.h>
#include <mission_planner_msgs/getCurrentPosition.h>
#include <mission_planner_msgs/telemetry.h>
#include <mission_planner_msgs/SensorPacket.h>

#include <ctime>
#include <stdlib.h> 
#include <cmath>

using namespace std;

class Drone
{

  private:

   ros::NodeHandle n;
   ros::Publisher n_pub;
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
      private_nh.param("latitude", lat, 45.97);
      private_nh.param("longitude",lng, 7.45);
      private_nh.param("altitude",alt, 15.00);

      

      n_sub = n.subscribe("drone", 1000, &Drone::catchMission, this);

      //service telemetry_activation
      telemetryActivation = n.advertiseService("telemetry_activation", &Drone::telemtryActivation , this);

	  //publisher feedback
  	  n_pub = n.advertise<mission_planner_msgs::SensorPacket>("feedback", 1000);


	}


    ~Drone(){
    }

	void catchMission(const mission_planner_msgs::Drone::ConstPtr& mission)
	{

      ROS_INFO("Travelling without moving... I am really not moving yeah (simulation only)");
	

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
