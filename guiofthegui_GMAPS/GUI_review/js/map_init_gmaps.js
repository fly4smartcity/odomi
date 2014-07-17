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

	var starting = new google.maps.LatLng(45.08367528971948, 7.605402247571433 ); //Torino 45.079006,7.61466 Nardò 

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
	                             new google.maps.LatLng(45.08367528971948,7.605402247571433),
	                             new google.maps.LatLng(45.0841027805813,7.608550382106472),
	                             new google.maps.LatLng(45.08418307153839,7.609234399399199),
	                             new google.maps.LatLng(45.08471860925961,7.60926484308621),
	                             new google.maps.LatLng(45.08554466641419,7.609188007435488),
				     new google.maps.LatLng(45.08618329907102,7.609228479862258),
				     new google.maps.LatLng(45.08636655505995,7.608785901639468),
				     new google.maps.LatLng(45.08659846645951,7.608999159013967),
				     new google.maps.LatLng(45.08680013451648,7.608578272930322),
				     new google.maps.LatLng(45.08690077412707,7.607808830398513),
				     new google.maps.LatLng(45.0869344996978,7.607569364625642),
				     new google.maps.LatLng(45.08677432764915,7.607522697990346),
				     new google.maps.LatLng(45.08681112337425,7.607282730402845),
				     new google.maps.LatLng(45.08750253711596,7.602184556422387),
				     new google.maps.LatLng(45.08774136201727,7.596370752933257),
				     new google.maps.LatLng(45.08663759506856,7.591239969561931),
				     new google.maps.LatLng(45.08630518366243,7.593551233432462),
				     new google.maps.LatLng(45.08541313954995,7.593303737728812),
				     new google.maps.LatLng(45.08367528971948,7.605402247571433),
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

