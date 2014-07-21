// Connecting to ROS
  // -----------------

var ros_pub_msg = new ROSLIB.Ros({
	//url : 'ws://192.168.1.88:9090'
	  url : 'ws://localhost:9091'
  });

function publ_waypoint(p)
{
  // Publishing a Topic
  // ------------------

  var pubwp = new ROSLIB.Topic({
    ros : ros_pub_msg,
    name : '/gui_waypoints',
    messageType : 'mission_planner_msgs/CoordinateArray'
  });

  var gps_way = Array();
  var elem;
  for(var i = 0; i < (p.length); i++)
  {

   elem = new Object();
   
   
   elem.latitude = parseFloat(p[i].latitude) ;
   elem.longitude = parseFloat(p[i].longitude) ;
   elem.heading  = 0;
   
   gps_way[i] = elem;

  }	  

  var pwayp = new ROSLIB.Message({
		 
	  waypoint : gps_way
	
  });
  
  pubwp.publish(pwayp);

}

function publ_home()
{
  // Publishing a Topic
  // ------------------

  var pubhome = new ROSLIB.Topic({
    ros : ros_pub_msg,
    name : '/teleop',
    messageType : 'mission_planner_msgs/Teleop'
  });


  var home = new ROSLIB.Message({
		 
	  type : 1
	
  });
  
  pubhome.publish(home);

}

function publ_circle()
{
  // Publishing a Topic
  // ------------------

  var pubhome = new ROSLIB.Topic({
    ros : ros_pub_msg,
    name : '/teleop',
    messageType : 'mission_planner_msgs/Teleop'
  });


  var circle = new ROSLIB.Message({
		 
	  type : 2
	
  });
  
  pubhome.publish(circle);

}
