//instanzio le variabili di controllo eventi
	var scegliTarget=false;//flag per la scelta del target sulla mappa
	var scegliMoltiTarget=false;//flag per la scelta di molti target sulla mappa
	var creaPercorso=false;//(vedi funzioni savePath,createPath): abilita/disabilita click per point sulla mappa nella funzione onMapClick/onMarkerClick
	var modificaPercorso=false;//(vedi funzioni savePath,createPath): abilita/disabilita click per point sulla mappa nella funzione onMapClick/onMarkerClick


//mie funzioni	
	function getlat(v)//date le coordinate ritorna la latitudine
	{
		var x=v.toString().split(",");
		var lat= x[0].toString();
		lat=lat.split("(");
		return(lat[1]);		
	}
	function getlng(v)//date le coordinate ritorna la longitudine
	{
		var x=v.toString().split(",");
		var lng= x[1].toString();		
		lng=lng.split(")");
		return(lng[0]);	
	}
	
	//alert(getlng(home.getLatLng()));//per ottenere lat e lng di un marker
	
	function updateMarker(marker,newLat,newLng)//simula lo spostamento dei marker
	{
		//alert("update marker lat:"+newLat+" lng:"+newLng+" coor old: "+marker.getLatLng());		
		var newLatLngMrk= new L.LatLng(newLat, newLng);//istanzio le nuove coordinate
		marker.setLatLng(newLatLngMrk);//applico le nuove coordinate	
		//alert(" coor new: "+marker.getLatLng());
	}
	function deleteMarker(mrk,map)//cancella un mrk(visto come layer) su una mappa 
	{
		map.removeLayer(mrk);
	}
	var pathMarker = L.AwesomeMarkers.icon({
          icon: 'circle',
          markerColor: 'blue',
          prefix: 'fa'
        });
	function drawPath(map,path)//disegna un percorso su mappa da un path di punti e ritorna il layer del percorso
	{
		var i;
		//var provMarker;
		var layer= new Array();	//creo il vettore di layer dei marker del percorso
		for(i=0;i<path.length;i++)//inserisco
		{
			//alert("ciclo draw "+path[i].lat+" "+ path[i].lng);
			layer[i] = new L.marker([path[i].lat, path[i].lng], {icon: pathMarker}).setZIndexOffset(0)/*.on('click',onMarkClick)*/;	
			//alert(layer[i].getLatLng());
    			map.addLayer(layer[i]);			
		}			
		return layer;//ritorna il layer dei marker
	}
	function erasePath(map,layer)//cancella un percorso disegnato su mappa visto come vettore di layer
	{		
		var i;		
		for(i=0;i<layer.length;i++)//per ogni elemento del mio percorso
		{	
			//alert("cancello");
			map.removeLayer(layer[i]);			
		}
	}
	function loadPath(map)//carica percorso da DB e ritorna il layer di punti
	{
		var layer;
		var path= new Array();	
		//simulo l'acquisizione da DB
		var i;
		var lt=45.05;
		var lg=7.664;
		for(i=0;i<5;i++)//inserisco
		{
			//alert("ciclo");
			path[i]= new Object();//creo oggetto			
			path[i].lat=lt;
			path[i].lng=lg;
			lt+=0.1;
			lg+=0.1;
		}	
		//
		/*for(i=0;i<5;i++)//stampo per prova
		{
			alert("lat"+ i +" "+path[i].lat+" lng"+ i +" "+path[i].lng);			
		}*/	
		layer=drawPath(map,path);
		return layer;
	}
	function getPathdatoLayer(layer)
	{
		var path= new Array();
		var n=layer.length;
		for(var i=0;i<n;i++)//inserisco
		{
			path[i]= new Object();//creo oggetto			
			path[i].latitude=getlat(layer[i].getLatLng());
			path[i].longitude=getlng(layer[i].getLatLng());
			path[i].altitude=0.0;
			path[i].heading=0.0;
		}
		return path;
	}
	function createPath()//crea un percorso disegnandolo su mappa, ritorna il layer del percorso
	{
		creaPercorso=true;
		alert("clicca una serie di punti per il tuo percorso");
	}
	
	function modifyPath(map)
	{
		modificaPercorso=true;
		//load	
	}
	function savePath(map,layer)
	{	
		var p;
		var n;
		var i;
		if (creaPercorso==true)
		{
			creaPercorso=false;
			p=getPathdatoLayer(layer);//estraggo il path GPS dai marker
			n=p.length;
			for(i=0;i<n;i++)
			{
				alert("GPS: "+p[i].lat +","+ p[i].lng);//salvo sul db le coordinate GPS del percorso
			}
			alert("percorso salvato");
		}
		else if (modificaPercorso==true)
		{
			modificaPercorso=false;
			p=getPathdatoLayer(layer);//estraggo il path GPS dai marker
			n=p.length;
			for(i=0;i<n;i++)
			{
				alert("GPS: "+p[i].lat +","+ p[i].lng);//salvo sul db le coordinate GPS del percorso
			}
			alert("percorso aggiornato");
		}
	}
	/******************************
	mette  un marker in funzione dove si vuole cliccare, ritorna un vettore di marker
	*********************************/
	function getIdMarker(LatLng,layer)//ritorna la posizione dell'marker nel vettore layer date le coordinate del punto, -1 se non c'è,funzione utilizzata in on map click per vedere se il punto cliccato e' un marker in Vlayer
	{
		var i;
		var ris=-1;		
		for(i=0;i<layer.length && ris==-1;i++)
		{			
			if (LatLng==layer[i].getLatLng())
			{
				ris=i;
			}
		}
		return ris;
	}
	function eliminaDatoID(id,layer)//elimina da layer la posizione id e compatta il vettore
	{
		var i;
		for (i=id; i<layer.length-1; i++)//faccio slittare il vett di 1
		{
			layer[i]=layer[i+1];
		}
		i=layer.pop();	
		//elimino l'ultima posizione che e' vuota
		
	}

