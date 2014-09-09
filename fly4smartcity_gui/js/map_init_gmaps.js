	//Buttons
	// "Avvia mission" disabled
	$('#btnGoMission').prop('disabled', true);
	// "Invia Percorso" disabled
	$('#btnInviaPercorso').prop('disabled', true);
	$('#btnCancellaPercorso').prop('disabled', true);

	
	var c = 0;
	var cx = 0;
	var map;
	var path = Array();
	var vectorMarker = Array();
	var vectorMarkerWp = Array();

	var markerHome = 0;
	var markerTarget = 0;
	var markerCurrentPosition = 0;
	var oldCurrentPosition = Array();
	var creapercorso = false;

    function initialize() {

	var starting = new google.maps.LatLng(45.066278, 7.688945 ); //Torino 45.079006,7.61466 Nardò 

        var mapOptions = {
          center: starting,
          zoom: 15,
	  mapTypeId: 'satellite'

		  
		  

        };

    map = new google.maps.Map(document.getElementById("map-canvas"),mapOptions);
	map.setTilt(45);

	
	google.maps.event.addListener(map, 'click', function(event) { //onClick
		if(creapercorso == true){//onClick
			
			//this is for multitarget
			if(c < 5) placeMarker(event.latLng,map);
			else
				alert("You've run out of targets!");
			
			//Buttons
			$('#btnInviaPercorso').prop('disabled', false);
			$('#btnCancellaPercorso').prop('disabled', false);
		}
	});

	var bufferZone = 		   [
	                 		    /*
	                             new google.maps.LatLng(  45.0662689209,  7.68884277344),
	                             new google.maps.LatLng(45.0662574768, 7.68884992599),
	                             new google.maps.LatLng(45.0662307739, 7.68891811371),
	                             new google.maps.LatLng(         45.0662384033,7.68893480301),
	                             new google.maps.LatLng(         45.0662956238 ,7.68897390366),
				     new google.maps.LatLng(         45.0663108826 , 7.68896579742),
				     new google.maps.LatLng(         45.0663299561,7.68890714645),
				     new google.maps.LatLng(         45.0663261414 ,7.68888521194),
				     new google.maps.LatLng(         45.0662689209 ,7.68884277344),*/
	                           ];

	var flightZone = 			[
	                 			/* new google.maps.LatLng(45.084407,7.608691),
	                 			 new google.maps.LatLng(45.085296,7.608932),
	                 			 new google.maps.LatLng(45.085555,7.607053),
	                 			 new google.maps.LatLng(45.084670,7.606823),
	                 			 new google.maps.LatLng(45.084407,7.608691) */
                      			];
	
	var bufferPath = new google.maps.Polyline({
	                             path: bufferZone,
	                             geodesic: true,
	                             strokeColor: '#FF0000',
	                             strokeOpacity: 1.0,
	                             strokeWeight: 2
	                           });
	var flightPath = new google.maps.Polyline({
								path: flightZone,
								geodesic: true,
								strokeColor: '#00FF00',
								strokeOpacity: 1.0,
								strokeWeight: 2
      							});
	
	bufferPath.setMap(map);
	flightPath.setMap(map);
      	}

    
    function visualizeOpendata (odObj,h,label) { //visualize an array of open data vectors 
		
    	
    	for(var i = 0; i < odObj.length; i++){
    		
    		odZone = odObj[i]; 
    		var helem = 0.5;//h[i]/5.0;
    		console.log("lat " + odObj[i][0]);

    		var od ;
    		
    	if(label == "edifici")  od = new google.maps.Polygon({
            path: odZone,
            geodesic: true,
            strokeColor: '#FF0000',
            fillColor: '#FF0000',
            fillOpacity: helem,
          });
  	
    	if(label == "alberate") od = new google.maps.Circle({
    		  strokeColor: '#00FF00',
    	      strokeOpacity: 0.8,
    	      strokeWeight: 2,
    	      fillColor: '#00FF00',
    	      fillOpacity: 0.35,
    	      map: map,
    	      center: odObj[i][0],
    	      radius: 4
          });
    	
    	od.setMap(map);
    	}
	}
    
	function placeMarker(position, map) {
	  
	  var multiTarget = ['images/marker1.png','images/marker2.png','images/marker3.png','images/marker4.png','images/marker5.png' ];
		  
	  var marker = new google.maps.Marker({
	    position: position,
	    map: map,
	    title: c.toString(),
	    icon : multiTarget[c]
	  });
	  
	  
	  //erasing function ()
	  vectorMarker[c] = marker;
	  
	  //fill the vector path
	  
	  var latlonObj = new Object();
	  
	  latlonObj.latitude = position.lat();
	  latlonObj.longitude = position.lng();
	  
	  path[c] = latlonObj;
	  
	  //alert(path[c].latitude + " " + path[c].longitude + "index " + c);
	  
	  c++;
	  }
	 
	function placeMarkerWp(lat, lng) {
		  
		  //var wpmultiTarget = ['images/marker1.png','images/marker2.png','images/marker3.png','images/marker4.png','images/marker5.png' ];
		  var position = new google.maps.LatLng(lat,lng);
		  
		  var marker = new google.maps.Marker({
		    position: position,
		    map: map,
		    title: cx.toString(),
		    //icon : wpmultiTarget[cx]
		  });
		  
		  vectorMarkerWp[cx] = marker;		  
		  cx++;
		  }
	  
	
	
	function erasePath()//delete markers
	{		
		for (var i = 0; i < vectorMarker.length; i++ ) {
			vectorMarker[i].setMap(null);
			if (cx != 0)vectorMarkerWp[i].setMap(null);

			path[i] = null;
		  }
		vectorMarker.length = 0;
		c = 0;
	}
	
	function putHome(lat, lng)
	{
		var position = new google.maps.LatLng(lat,lng);
		var Home = 'images/homeMarker.png';
		  
		    markerHome = new google.maps.Marker({
		    position: position,
		    map: map,
		    title: "Home",
		    icon : Home
		  });
		  
		  
	}
	
	function putTarget(lat, lng)
	{
		var position = new google.maps.LatLng(lat,lng);
		var Target = 'images/targetMarker.png';
		  
		    markerTarget = new google.maps.Marker({
		    position: position,
		    map: map,
		    title: "Target",
		    icon : Target
		  });
		  
		  
	}
	
	function putCurrentPosition(lat, lng)
	{
		var position =  new google.maps.LatLng(lat,lng);
		var CurPos = 'images/currentPositionMarker1.png';
		  
		    markerCurrentPosition = new google.maps.Marker({
		    position: position,
		    map: map,
		    title: "Drone",
		    icon : CurPos
		  });
  
	}
	
	function deleteHome() {
		
		if(markerHome != 0)
		markerHome.setMap(null);
	}
	
	function deleteTarget() {
		
		if(markerTarget != 0)
		markerTarget.setMap(null);		
	}
	
	function deleteCurPos() {
		
		if(markerCurrentPosition != 0)
		markerCurrentPosition.setMap(null);		
	}
	
	
	function drawPath(arrayOfPositions) {
		
		var flightPath = new google.maps.Polyline({
		    path: arrayOfPositions,
		    geodesic: true,
		    strokeColor: '#FFFF00',
		    strokeOpacity: 1.0,
		    strokeWeight: 2
		  });

		  flightPath.setMap(map);
		
	}
      google.maps.event.addDomListener(window, 'load', initialize);

