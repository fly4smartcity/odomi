

var Vlayer=new Array();//vettore di layer di punti
//coordinate gps del target da inviare
var targetLat;
var targetLng;
var creapercorso = false;
var multiLat = new Array();
var multiLng = new Array();
var ix = 0;
var p;
//
	

	var map = L.map('map').setView([45.079006,7.61466], 14);//ISTANZIA LA MAPPA CON GPS E ZOOM
/*
 * https://{s}.tiles.mapbox.com/v3/{id}/{z}/{x}/{y}.png', {
			maxZoom: 18,
			attribution: 'Map data &copy; <a href="http://openstreetmap.org">OpenStreetMap</a> contributors, ' +
				'<a href="http://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, ' +
				'Imagery © <a href="http://mapbox.com">Mapbox</a>
 *  */
	//L.tileLayer(CM_URL, {attribution: CM_ATTR, styleId: 997}).addTo(map);//da lo sfondo
	L.tileLayer('http://{s}.tile.osm.org/{z}/{x}/{y}.png', {
	    attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
			id: 'examples.map-9ijuk24y' , maxZoom: 25,
		}).addTo(map);
	
/***********************
definizione del cerchio : centroGPS,raggio,stile
************************/

/**************************
definizione del poligono dando i valori in senzo orario
**************************/





	
	function onMarkClick(e){//se viene click un marker viene richiamata questa funzione che permette di elaborarlo in funzione di cosa si sta' facendo		
		var gps=e.latlng.toString();
		if (creaPercorso==true)	//se sono in modalita' crea percorso
		{
			//alert("id e' "+getIdMarker(e.latlng.toString()));
			map.removeLayer(Vlayer[getIdMarker(gps,Vlayer)]);//elimino il marker dalla mappa
			//alert("eliminato");
			eliminaDatoID(getIdMarker(gps,Vlayer),Vlayer);//elimino il marker dal vettore
			//il refresh dei punti viene in automatico		
		}
	}
	function onMapClick(e,markerArray,maxLenght,lenght) 		{		
		
		if(creapercorso == true){
		var gpsClick=e.latlng.toString();
		var i;
		//alert(creaPercorso+" "+gpsClick+" "+Vlayer.length);
				
			i=Vlayer.length;
			
			Vlayer[i]= new L.marker([getlat(gpsClick), getlng(gpsClick)]).on('click',onMarkClick);//inserisco il punto nel vettore
			map.addLayer(Vlayer[i]);//lo disegno sulla mappa
			
			//estraggo il path GPS dai marker
			p=getPathdatoLayer(Vlayer);//estraggo il path GPS dai marker

			n=p.length;
				
			
			$('#btnInviaPercorso').prop('disabled', false);
			$('#btnCancellaPercorso').prop('disabled', false);

	}

	}
	

/*************************
evento on click
**************************/
	map.on('click', onMapClick);




//marker con font awesome per la home, destinazione e drone
var homeMarker = L.AwesomeMarkers.icon({
    icon: 'home',
    markerColor: 'darkpurple',
    prefix: 'fa'
  });
var goalMarker = L.AwesomeMarkers.icon({
    icon: 'flag-checkered ',
    markerColor: 'green',
    prefix: 'fa'
  });
var droneMarker = L.AwesomeMarkers.icon({
    icon: 'plane',
    markerColor: 'red',
    prefix: 'fa'
  });
//
//L.marker([45.07, 7.66], {icon: redMarker}).addTo(map);
//ISTANZIO LA HOME E IL DRONE
//ZIndexOffset mette il numero del livello del layer a cui si riferisce il marker
	var home=new L.marker([0.0, 0.0], {icon: homeMarker}).bindPopup("I am Home").setZIndexOffset(1);//.addTo(map)
	var drone=new L.marker([0.0, 0.0], {icon: droneMarker}).bindPopup("I am a DRONE.").setZIndexOffset(2);//.addTo(map)
	var goal=new L.marker([0.0, 0.0], {icon: goalMarker}).bindPopup("I am the GOAL.").setZIndexOffset(1);

	
	// "Avvia mission" disabled
	$('#btnGoMission').prop('disabled', true);
	// "Invia Percorso" disabled
	$('#btnInviaPercorso').prop('disabled', true);
	$('#btnCancellaPercorso').prop('disabled', true);


/*
$('#btnHome').prop('disabled', true);
$('#btnGoMission').prop('disabled', true);
$('#btnTarget').prop('disabled', true);
$('#btnDefaulTarget').prop('disabled', true);
$('#btnMultiTarget').prop('disabled', true);*/

