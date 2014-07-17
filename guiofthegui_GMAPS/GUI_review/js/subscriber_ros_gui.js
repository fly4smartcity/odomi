  // Connecting to ROS
  // -----------------
  var index = 0 ;
  var i = 0;
  var d = new Date();
  var time  = d.getTime();
  var timeM = time;
  
  var ros = new ROSLIB.Ros({
	  //url : 'ws://192.168.1.88:9090'
    	url : 'ws://localhost:9091'
  });

  // Subscribing to a Topic
  // ----------------------

  var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/feedback',
    messageType : 'mission_planner_msgs/SensorPacket'
   
  });
  

  
  listener.subscribe(function(message) {
    	  
    // Attitude update
    attitude.setRoll(message.roll_ang);
    attitude.setPitch(message.pitch_ang);
    
    // Heading update
    heading.setHeading(message.compass_heading);
    
    // Airspeed update -> becomes battery in Volts
    if(message.battery > 0)
    airspeed.setAirSpeed(message.battery); // 16.8 V -> maxVolt 13.8-> minVolt
    else
    	airspeed.setAirSpeed(0);
    //alert(message.altimeter);
    if(message.altimeter > 0)
    	
    altimeter.setAltitude(message.altimeter*100); //dam
    else
    	airspeed.setAltitude(0);

    if(index == 0){ // firts time
    	putCurrentPosition(message.c_latit,message.c_longit);
        putHome(message.h_latit,message.h_longit);

    	oldCurrentPosition[index] = new google.maps.LatLng(message.c_latit,message.c_longit);

    }else
    	{ // new entries
    var newpositionC = new google.maps.LatLng(message.c_latit,message.c_longit);

	markerCurrentPosition.setPosition(newpositionC);
	
	d = new Date();
	time  = d.getTime();
		  	
	
	if((time - timeM) >= 200)
		{
	oldCurrentPosition[i] = newpositionC;
	drawPath(oldCurrentPosition);
	
	timeM = time;
	i++;
    	}
    
    }
    index++; // update index
  });

//subscribing per messaggi target
  var listener_target = new ROSLIB.Topic({
    ros : ros,
    name : '/mp_waypoints',
    messageType : 'mission_planner_msgs/CoordinateArray'

  });
  
  listener_target.subscribe(function(message) {
	  
	  var ll = message.waypoint.length;
	  
	  //home
      putHome(message.waypoint[0].latitude,message.waypoint[0].longitude);

	  
	  //target 
	  deleteTarget();	  

	  putTarget(message.waypoint[ll-1].latitude,message.waypoint[ll-1].longitude);
	  
	  //waypoints
	  for(var j=1; j<ll-1; j++)
	  {
		
		placeMarkerWp( message.waypoint[j].latitude,message.waypoint[j].longitude);
	  }
	  
//alert("target  " + message.waypoint[ll-1].latitude + ","+ message.waypoint[ll-1].longitude + "  circle   " +  message.waypoint[j-1].latitude + "," + message.waypoint[j-1].longitude);
});


  //subscribing per check
  var listener_check = new ROSLIB.Topic({
    ros : ros,
    name : '/check',
    messageType : 'std_msgs/Bool'

  });
  
  
  listener_check.subscribe(function(message) {

  if(message.data == true)
  $('#btnGoMission').prop('disabled', false);
  else
	  $('#btnGoMission').prop('disabled', true);

  });
  
  
	  
  