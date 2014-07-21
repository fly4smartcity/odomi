/* reference
 http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality
*/

function call_srv(lat,lng)
{
  // Connecting to ROS
  // -----------------
 var ros = new ROSLIB.Ros({
	// url : 'ws://192.168.1.88:9090'
	 url : 'ws://localhost:9091'
  });
  var targetClient = new ROSLIB.Service({
    ros : ros,
    name : '/build_mission',
    serviceType : 'mission_planner_msgs/getCurrentPosition'
  });

  var request = new ROSLIB.ServiceRequest({
    latitude : lat,
    longitude : lng
  });

  targetClient.callService(request, function(result) {    
  });  
}

function takeoff_activation()
{
	 // Connecting to ROS
	  // -----------------
	 var ros = new ROSLIB.Ros({
		 url : 'ws://localhost:9091'
	  });
	  var targetClient = new ROSLIB.Service({
	    ros : ros,
	    name : '/takeoff_activation',
	    serviceType : 'mission_planner_msgs/TakeoffActivation'
	  });

	  var request = new ROSLIB.ServiceRequest({
	    activate : 2128
	  });

	  targetClient.callService(request, function(result) {    
	  });  
	

}