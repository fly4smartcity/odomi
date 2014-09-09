#include "ros/ros.h"
#include <mission_planner_msgs/Drone.h>
#include <mission_planner_msgs/Coordinate.h>
#include <mission_planner_msgs/CoordinateArray.h>
#include <mission_planner_msgs/getCurrentPosition.h>
#include <mission_planner_msgs/telemetry.h>
#include <mission_planner_msgs/SensorPacket.h>

#include <ctime>
#include <math.h>
#include <float.h>

#define pi 3.14159265358979323846


using namespace std;

class MissionPlanner
{
  public:
    MissionPlanner()
    {
      ros::NodeHandle private_nh("~");
      h_lat=0.0f;
      h_lon=0.0f;

      private_nh.param("no_feedback_max_time", max_time, 5);

      mission_service = n.advertiseService("build_mission", &MissionPlanner::buildmissionCallback, this);

      // Latched publisher for drone
      mission_pub= n.advertise<mission_planner_msgs::Drone>("drone", 1, false);

      waypoint_pub= n.advertise<mission_planner_msgs::CoordinateArray>("mp_waypoints", 1, false);

      timer = n.createTimer(ros::Duration(1.0), &MissionPlanner::timerCallback, this);

      sensor = n.subscribe("feedback", 1, &MissionPlanner::feedbackCallback, this);
      //gui_waypoints = n.subscribe("gui_waypoints", 1, &MissionPlanner::waypointCallback, this);
      waypoints = n.subscribe("waypoints", 1, &MissionPlanner::waypointCallback, this);

      telemetry_activated = false;

      //mutex1 = PTHREAD_MUTEX_INIZIALIZER;
      time0 = time(NULL);
      diff = abs(time(NULL) - time0);

      first_=true;


    }

    ~MissionPlanner(){

    }

    double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
  double theta, dist;
  theta = lon1 - lon2;

  dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515;
  switch(unit) {
    case 'M':
      break;
    case 'K':
      dist = dist * 1.609344;
      break;
    case 'N':
      dist = dist * 0.8684;
      break;
  }
  return (dist);
  }

  /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  /*::  This function converts decimal degrees to radians             :*/
  /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  double deg2rad(double deg) {
  return (deg * pi / 180);
  }

  /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  /*::  This function converts radians to decimal degrees             :*/
  /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  double rad2deg(double rad) {
  return (rad * 180 / pi);
	}


    mission_planner_msgs::Coordinate calculate_target_on_circle (mission_planner_msgs::Coordinate home, mission_planner_msgs::Coordinate target, double radius){

	mission_planner_msgs::Coordinate new_target;
	double R = 6378137;
	double PI = 3.14159265358979323846;
	double val1, val2, val3, val4, val5;

	double home_lat_rad = home.latitude *(PI/180);
	double home_lon_rad = home.longitude *(PI/180);

	ROS_INFO("Home  in radianti lat = %f, long = %f", home_lat_rad, home_lon_rad);

	double target_lat_rad = target.latitude *(PI/180);
	double target_lon_rad = target.longitude *(PI/180);

	ROS_INFO("Target  in radianti lat = %f, long = %f", target_lat_rad, target_lon_rad);

	double dlon = target_lon_rad - home_lon_rad;
	double dlat = target_lat_rad - home_lat_rad;

	val1 = 	sin(dlat/2)*sin(dlat/2);
	val2 = 	cos(home_lat_rad);
	val3 =  cos(target_lat_rad);
	val4 =  val2*val3;
	val5 = sin(dlon/2)*sin(dlon/2);

	ROS_INFO("Val1 = %2.30f, val2 = %2.30f, val3 = %2.30f, val4 = %2.30f, val5 = %2.30f",val1, val2, val3, val4, val5);

	double a = (val1 + val4*val5);

	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	double d = R * c;
	double rapporto = 1 - radius/d;

	ROS_INFO("a = %2.15f, c = %2.10f , d = %2.10f", a, c, d);


	new_target.latitude = dlat*(180/PI)*rapporto + home.latitude;
	new_target.longitude = dlon*(180/PI)*rapporto + home.longitude;
	new_target.altitude = target.altitude;

	ROS_INFO("New Target lat  = %f, long = %f", new_target.latitude, new_target.longitude);


        return new_target;



    }

    void waypointCallback(const mission_planner_msgs::CoordinateArrayConstPtr& coords){

	int size = coords->waypoint.size() + 4;

	//publish drone topic
	mission_planner_msgs::Drone drone;
	drone.header.stamp = ros::Time::now();
	drone.name = "Pro S3";
	drone.type_name = "Quadrocopter";
	drone.home.latitude = h_lat;
	drone.home.longitude = h_lon;
	drone.movements.resize(2 + size);

	// TAKE OFF
	drone.movements[0].name = "take off";
	drone.movements[0].type = 2;
	drone.movements[0].altitude=15.0;



	for(int i = 1; i < size-3 ; i++){

		// GO TO WAYPOINT
		drone.movements[i].name = "go to waypoint";
		drone.movements[i].type = 4;
		drone.movements[i].target_position.latitude=coords->waypoint[i-1].latitude;
		drone.movements[i].target_position.longitude=coords->waypoint[i-1].longitude;
		drone.movements[i].target_position.altitude=coords->waypoint[i-1].altitude;
		drone.movements[i].strategy = 0;


	}


	// HOVER
	/*drone.movements[size-3].name = "Hover and wait";
	drone.movements[size-3].type = 5;
	//drone.movements[size-3].duration = 3600;

	// GO TO LANDING POINT
	drone.movements[size-2].name = "go to landing point";
	drone.movements[size-2].type = 4;
	drone.movements[size-2].target_position.latitude=h_lat;
	drone.movements[size-2].target_position.longitude=h_lon;
	drone.movements[size-2].target_position.altitude=15.0;
	drone.movements[size-2].strategy = 0;

	// LAND
	drone.movements[size-1].name = "land";
	drone.movements[size-1].type = 3;
	drone.movements[size-1].target_position.heading=0; */

        mission_pub.publish( drone );
	waypoint_pub.publish( coords );
   }


    void feedbackCallback(const mission_planner_msgs::SensorPacketConstPtr& packet){

        h_lat = packet->h_latit;
        h_lon = packet->h_longit;
       // pthread_mutex_lock(&mutex1);
       	time0 = time(NULL);
        //pthread_mutex_unlock(&mutex1);
	//ROS_INFO("set home position lon = %lf, lat = %lf", h_lon, h_lat);

    }


    bool activate_telemetry(){


        ros::ServiceClient telem_service = n.serviceClient<mission_planner_msgs::telemetry>("/telemetry_activation");
	mission_planner_msgs::telemetry srv;
	srv.request.activate = true;
	if(telem_service.call(srv))
		return srv.response.resp;
	return false;


    }


    void timerCallback(const ros::TimerEvent& e){

	if(!telemetry_activated){
		if(activate_telemetry()){
			ROS_INFO(" telemetry has been activated ");
			telemetry_activated = true;
			//timer.stop();
      			time0 = time(NULL);
		}else{
			if(first_)
				ROS_WARN(" telemetry not activated, try again in 1 second ... (the message won't be repeated)");
			first_=false;
		}
	}else{

      		diff = abs(time(NULL) - time0);
		if(diff > max_time){

			ROS_WARN(" telemetry not sensed for %d sec , try activating ...", diff);
			telemetry_activated = false;
			first_ = true;

		}
	}
    }


    bool buildmissionCallback(mission_planner_msgs::getCurrentPosition::Request  &req, mission_planner_msgs::getCurrentPosition::Response &res )
    {
	ROS_INFO("build mission with current pos: lat = %lf, lon = %lf", req.latitude, req.longitude);

	mission_planner_msgs::CoordinateArray coords;
	coords.waypoint.resize(3);

	//publish target topic
	mission_planner_msgs::Coordinate target;
	target.latitude = req.latitude;
	target.longitude = req.longitude;
	target.altitude = 15.0;
        //target_pub.publish( target );

        mission_planner_msgs::Coordinate home;
	home.latitude = h_lat;
	home.longitude = h_lon;
	home.altitude = 15.0;

        coords.waypoint[0].latitude = h_lat;
        coords.waypoint[0].longitude = h_lon;
        coords.waypoint[0].altitude = home.altitude;

	double radius = 10.0;

        mission_planner_msgs::Coordinate new_target = calculate_target_on_circle(home, target, radius);

	//publish drone topic
	mission_planner_msgs::Drone drone;
	drone.header.stamp = ros::Time::now();
	drone.name = "Pro S3";
	drone.type_name = "Quadrocopter";
	drone.home.latitude = h_lat;
	drone.home.longitude = h_lon;
	drone.movements.resize(7);

	// TAKE OFF
	drone.movements[0].name = "take off";
	drone.movements[0].type = 2;
	drone.movements[0].altitude=15.0;

	// GO TO  RESCUE POINT
	drone.movements[1].name = "go to point on circle";
	drone.movements[1].type = 4;
	drone.movements[1].target_position.latitude=new_target.latitude;
	drone.movements[1].target_position.longitude=new_target.longitude;
	drone.movements[1].target_position.altitude=new_target.altitude;
	drone.movements[1].strategy = 0;

        coords.waypoint[1].latitude = new_target.latitude;
        coords.waypoint[1].longitude = new_target.longitude;
        coords.waypoint[1].altitude = new_target.altitude;

	// HOVER
	drone.movements[2].name = "Hover and wait";
	drone.movements[2].type = 5;
	//drone.movements[2].duration = 3600;

	// CIRCLING WITH A SET RADIUS
	drone.movements[3].name = "Circling with a set radius";
	drone.movements[3].type = 6;
	drone.movements[3].target_position.latitude=target.latitude;
	drone.movements[3].target_position.longitude=target.longitude;
	drone.movements[3].target_position.altitude=target.altitude;
	drone.movements[3].radius = radius;

        coords.waypoint[2].latitude = target.latitude;
        coords.waypoint[2].longitude = target.longitude;
        coords.waypoint[2].altitude = target.altitude;

	// HOVER
	drone.movements[4].name = "Hover and wait";
	drone.movements[4].type = 5;
	//drone.movements[4].duration = 3600;

	// GO TO LANDING POINT
	drone.movements[5].name = "go to landing point";
	drone.movements[5].type = 4;
	drone.movements[5].target_position.latitude=h_lat;
	drone.movements[5].target_position.longitude=h_lon;
	drone.movements[5].target_position.altitude=15.0;
	drone.movements[5].strategy = 0;

	// LAND
	drone.movements[6].name = "land";
	drone.movements[6].type = 3;
	drone.movements[6].target_position.heading=0;


        mission_pub.publish( drone );
        waypoint_pub.publish( coords );
 	res.res = true;
	return true;
    }

private:

    ros::NodeHandle n;
<<<<<<< HEAD
    ros::NodeHandle private_nh;
=======
>>>>>>> 9ee2ed5823151f80049aa73bcf5d8c240b83eecf
    ros::Publisher mission_pub, target_pub, waypoint_pub;
    ros::ServiceServer mission_service;
    ros::Timer timer;
    ros::Subscriber sensor, gui_waypoints, waypoints;
    double h_lat;
    double h_lon;
    bool telemetry_activated, first_;
    time_t diff, time0;

   int max_time;
   // pthread_mutex_t mutex1;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mission_planner");
  MissionPlanner mp;
  ros::spin();

  return 0;
}
