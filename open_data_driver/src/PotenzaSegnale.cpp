
#include <string>
#include <iostream>
#include <sstream>
#include <cmath>
#include <opendata/PotenzaSegnale.h>
#include "open_data_msg/Data.h"
#include "open_data_msg/Open_data.h"

PotenzaSegnale::PotenzaSegnale(){}
PotenzaSegnale::~PotenzaSegnale(){}


open_data_msg::Data PotenzaSegnale::getPotenzaSegnaleFromBoundingBox(float x1, float y1 , float x2 , float y2)
{
	open_data_msg::Data signal;
	string url_base = "http://api.opensignal.com/v2/networkrank.json?";
	string apikey="&apikey=9d9e14b67555604e1583def1f471831d";
	int networkType = 4; //4G

	float lat,lng,distance,distanceLat,distanceLng;
	float x1_rad,x2_rad,y1_rad,y2_rad;
	//calcolo delle coordinate del punto medio
	lng = (x1+x2)/2;
	lat = (y1+y2)/2;


	//calcola distanza tra 2 punti date le loro longitudini e latitudini

	const float R = 6372.795477598;//raggio terrestre in Km

	 //trasformo in radianti
	 x1_rad=(x1*M_PI)/180;
	 x2_rad=(x2*M_PI)/180;

	 y1_rad=(y1*M_PI)/180;
	 y2_rad=(y2*M_PI)/180;
	 //lonA = x1 ; lonB = x2;
	 //latA = y1 ; latB = y2;

	 //distanza (A,B) = R * arccos(sin(latA) * sin(latB) + cos(latA) * cos(latB) * cos(lonA-lonB))

	 //considero due punti aventi stessa longitudine e diversa latitudine
	 //lonA = lonB = x1;
	 //cos(x1-x1) = cos(0) = 1
	 distanceLat = R * acos(sin(y1_rad) * sin(y2_rad) + cos(y1_rad) * cos(y2_rad) * 1);

	 //considero due punti aventi stessa latitudine e diversa longitudine
	 //latA = latB = y1;
	 //sin^2(y1)+cos^2(y1) = 1
	distanceLng  = R * acos(1 * cos(x2_rad - x1_rad));

	//distance è la distanza da punto medio al minimo(x o y) o al massimo(x o y)
	if(distanceLat > distanceLng)
		distance = distanceLat/2;
	else
		distance = distanceLng/2;


	// 4. with IOStreams
	stringstream sstm;
	sstm << url_base << "lat=" << lat << "&lng=" << lng <<"&distance=" <<distance  <<"&network_type=" << networkType << apikey;

	string url = sstm.str();

//return (ScriviSegnale(url));	
signal = ScriviSegnale(url);

	if(signal.data.size()>0)
	{
		if(x1 > x2)
	            {
	            	float tmp = x1;
	            	x1 = x2;
	            	x2 = tmp;
	            }

	            if(y1 > y2)
	            {
	            	float tmp = y1;
	            	y1 = y2;
	            	y2 = tmp;
	            }

		//assegno coordinate bbox in senso antiorario a partire da in basso a sx
		signal.data[0].area.points.resize(5); 
		signal.data[0].area.points[0].x = x1;
		signal.data[0].area.points[0].y = y1;
		signal.data[0].area.points[0].z = 0;
		signal.data[0].area.points[1].x = x1;
		signal.data[0].area.points[1].y = y2;
		signal.data[0].area.points[1].z = 0;
		signal.data[0].area.points[2].x = x2;
		signal.data[0].area.points[2].y = y2;
		signal.data[0].area.points[2].z = 0;
		signal.data[0].area.points[3].x = x2;
		signal.data[0].area.points[3].y = y1;
		signal.data[0].area.points[3].z = 0;
		signal.data[0].area.points[4].x = x1;
		signal.data[0].area.points[4].y = y1;
		signal.data[0].area.points[4].z = 0;

	}

	return signal;

}


open_data_msg::Data PotenzaSegnale::getPotenzaSegnale(float lng,float lat)
{
//string url = "http://api.opensignal.com/v1/networkrank.json?lat=45.1119&lng=7.7061&apikey=9d9e14b67555604e1583def1f471831d";

string url_base = "http://api.opensignal.com/v2/networkrank.json?";
string apikey="&apikey=9d9e14b67555604e1583def1f471831d";

// 4. with IOStreams
stringstream sstm;
sstm << url_base << "lat=" << lat << "&lng=" << lng << apikey;

string url = sstm.str();

return (ScriviSegnale(url));

}

open_data_msg::Data PotenzaSegnale::getPotenzaSegnale(float lng,float lat, int distance, int networkType)
{
Read rd;
//string url = "http://api.opensignal.com/v1/networkrank.json?lat=45.1119&lng=7.7061&apikey=9d9e14b67555604e1583def1f471831d";

string url_base = "http://api.opensignal.com/v2/networkrank.json?";
string apikey="&apikey=9d9e14b67555604e1583def1f471831d";

stringstream sstm;
sstm << url_base << "lat=" << lat << "&lng=" << lng <<"&distance=" << distance <<"&network_type=" << networkType << apikey;

string url = sstm.str();
return (ScriviSegnale(url));

}


open_data_msg::Data PotenzaSegnale::getPotenzaSegnale(int distance, float lng,float lat)
{
Read rd;
//string url = "http://api.opensignal.com/v1/networkrank.json?lat=45.1119&lng=7.7061&apikey=9d9e14b67555604e1583def1f471831d";

string url_base = "http://api.opensignal.com/v2/networkrank.json?";
string apikey="&apikey=9d9e14b67555604e1583def1f471831d";


stringstream sstm;
sstm << url_base << "lat=" << lat << "&lng=" << lng <<"&distance=" << distance << apikey;


string url = sstm.str();
return (ScriviSegnale(url));


}

open_data_msg::Data PotenzaSegnale::getPotenzaSegnale(float lng,float lat, int networkType)
{
Read rd;
//string url = "http://api.opensignal.com/v1/networkrank.json?lat=45.1119&lng=7.7061&apikey=9d9e14b67555604e1583def1f471831d";

string url_base = "http://api.opensignal.com/v2/networkrank.json?";
string apikey="&apikey=9d9e14b67555604e1583def1f471831d";

stringstream sstm;
sstm << url_base << "lat=" << lat << "&lng=" << lng <<"&network_type=" << networkType << apikey;

string url = sstm.str();
return (ScriviSegnale(url));

}

open_data_msg::Data PotenzaSegnale::ScriviSegnale(string s)
{
	Read rd;
	open_data_msg::Data signal;
	open_data_msg::Open_data op;
	string json_example = rd.read(s);

/*	string json_example = "{\"apiVersion\":\"1\",\"latitude\":\"45.1119\",\"longitude\":\"7.7061\",\"distance\":10,\"network_type\":null,\"perMinuteCurrent\":0,\"perMinuteLimit\":10,\"perMonthCurrent\":2,\"perMonthLimit\":2000,\"networkRank\":{\"vodafone IT\":{\"Type-3G\":{\"networkName\":\"vodafoneIT\",\"networkId\":\"22210\",\"networkType\":\"3\"," 
"\"averageRssi\":\"14.278305\",\"sampleSizeRSSI\":\"182981\",\"downloadSpeed\":\"2619.0500\",\"uploadSpeed\":\"887.4750\",\"pingTime\":\"242.4250\",\"reliability\":\"0.9750\"}}}}"; */

/*string json_example ="{\"apiVersion\": \"2\",\"latitude\": \"45.1119\",\"longitude\": \"7.7061\",\"distance\": \"10\",\"network_type\": \"4\",\"perMinuteCurrent\": 1, \"perMinuteLimit\":10,\"perMonthCurrent\": 9,\"perMonthLimit\": 2000,\"networkRank\": {\"network22210\": {\"type4G\": {\"networkName\": \"vodafone IT\",\"networkId\": \"22210\",\"networkType\": \"4\",\"averageRssiAsu\": \"18.402945\",\"averageRssiDb\": \"-103.194109\",\"sampleSizeRSSI\": \"66441\"}},\"network22201\": {\"type4G\": {\"networkName\": \"I TIM\",\"networkId\": \"22201\",\"networkType\": \"4\",\"averageRssiAsu\": \"21.176143\",\"averageRssiDb\": \"-97.647715\",\"sampleSizeRSSI\": \"9381\"}}}}";
*/


 // Let's parse it  
 Json::Value root;
 Json::Reader reader;
 bool parsedSuccess = reader.parse(json_example, 
                                   root, 
                                   false);
 if(!parsedSuccess)
 {	
	signal.status.status_code = -2;
	signal.status.reason = "Connessione al server non riuscita";	
	return signal;

 }
  
 // Let's extract the array contained 
 // in the root object
 Json::Value v = root["networkRank"];

	if(v.isString())
	{

	 	if(v.asString() == "No results for this area")
		{
		      signal.status.status_code = -1;
		      signal.status.reason = "Nessun risultato";	
		}
	}
		
	else
	{ 
	  Json::Value value = root["networkRank"]["network22201"]["type4G"]["averageRssiAsu"];

		if(!value.isNull())
		{	
		signal.status.status_code = 1;
		signal.status.reason = "OK";	

		 	signal.data.resize(1);
			signal.data[0].type = op.TYPE_DYNAMIC;            	 
			signal.data[0].label.assign("signal");  

		signal.data[0].attributes.resize(1);
		signal.data[0].attributes[0].key = "average Rssi assoluto";
		signal.data[0].attributes[0].value = value.asString();
		}
		
		else
		{
		      signal.status.status_code = -1;
		      signal.status.reason = "Nessun risultato";	
		}
	}

	return signal;
}



