/*
 * Weather.cpp
 *
 *  Created on: May 10, 2014
 *      Author: enrico
 */

#include <iostream>
#include <string>
#include <sstream>
#include <opendata/Weather.h>
#include "open_data_msg/Data.h"
#include "open_data_msg/Open_data.h"


Weather::Weather() {


}

Weather::~Weather() {

}
	

open_data_msg::Data Weather::getWeatherFromBuondingBox(float x1, float y1, float x2, float y2)
{
	Read rd;
	open_data_msg::Data weather;
	float lat,lng;

	//calcolo delle coordinate del punto medio
	lng = (x1+x2)/2;
	lat = (y1+y2)/2;

	string url_base = "http://api.openweathermap.org/data/2.5/weather?";
	stringstream sstm;
	sstm << url_base << "lat=" << lat <<"&lon=" << lng <<"&units=metric&lang=it";
	string url = sstm.str();
	weather = ScriviWeather(url);

	if(weather.data.size()>0)
	{


		//assegno coordinate bbox in senso antiorario a partire da in basso a sx
		weather.data[0].area.points.resize(5); 
		weather.data[0].area.points[0].x = x1;
		weather.data[0].area.points[0].y = y1;
		weather.data[0].area.points[0].z = 0;
		weather.data[0].area.points[1].x = x1;
		weather.data[0].area.points[1].y = y2;
		weather.data[0].area.points[1].z = 0;
		weather.data[0].area.points[2].x = x2;
		weather.data[0].area.points[2].y = y2;
		weather.data[0].area.points[2].z = 0;
		weather.data[0].area.points[3].x = x2;
		weather.data[0].area.points[3].y = y1;
		weather.data[0].area.points[3].z = 0;
		weather.data[0].area.points[4].x = x1;
		weather.data[0].area.points[4].y = y1;
		weather.data[0].area.points[4].z = 0;


	}

	return weather;


}


open_data_msg::Data Weather::ScriviWeather(string url)
{	  
	Read rd;
        open_data_msg::Data weather;
	open_data_msg::Open_data op;

	string json_string = rd.read(url);	
	float lat,lon,temp,humidity,wind_speed,wind_degree,clouds,rain,snow;
	const int mandatory_fields=5;
	int attribute_fields= mandatory_fields;
	
 	// Let's parse it
	 Json::Value root,optional,error,value;
	 Json::Reader reader;
	 
	bool parsedSuccess = reader.parse(json_string,
	                                   root,
	                                   false);
	 if(!parsedSuccess)
	 {
		weather.status.status_code = -2;
		weather.status.reason = "Connessione al server non riuscita";	
	  return weather;
	 }

	 // Let's extract the array contained
	 // in the root object
	
	if((error = root["coord"]).isNull())
	{
          weather.status.status_code = -1;	      
       	  weather.status.reason = "Nessun risultato";	
	  return weather;
	}
	

		
else
{ 
	 lat = root["coord"]["lat"].asDouble();
	 lon= root["coord"]["lon"].asDouble();

	 temp = root["main"]["temp"].asDouble();
	 humidity = root["main"]["humidity"].asDouble();
	 wind_speed = root["wind"]["speed"].asDouble();
	 wind_degree = root["wind"]["deg"].asDouble();
	 clouds = root["clouds"]["all"].asDouble();

	 if(!(optional = root["rain"]["3h"]).isNull())
		 rain = root["rain"]["3h"].asDouble();
	 if(!(optional = root["snow"]["3h"]).isNull())
	 		 snow = root["snow"]["3h"].asDouble();


	weather.status.status_code = 1;
	weather.status.reason = "OK";	

	 	weather.data.resize(1);
		weather.data[0].type = op.TYPE_DYNAMIC;            	 
		weather.data[0].label.assign("meteo");  
 
	weather.data[0].attributes.resize(attribute_fields);    //5 sono i campi obbligatori
	weather.data[0].attributes[0].key = "Temperature °C";
        stringstream stemp;
	stemp << temp;
	weather.data[0].attributes[0].value = stemp.str();
		
	weather.data[0].attributes[1].key = "Humidity %";
        stringstream shumidity;
	shumidity << humidity;
	weather.data[0].attributes[1].value = shumidity.str();

	weather.data[0].attributes[2].key = "Wind Speed m/s";
        stringstream swind_speed;
	swind_speed << wind_speed;
	weather.data[0].attributes[2].value = swind_speed.str();

	weather.data[0].attributes[3].key = "Wind Degree °";
        stringstream swind_degree;
	swind_degree << wind_degree;
	weather.data[0].attributes[3].value = swind_degree.str();

	weather.data[0].attributes[4].key = "Clouds %";
        stringstream sclouds;
	sclouds << clouds;
	weather.data[0].attributes[4].value = sclouds.str();

	 if(!(optional = root["rain"]["3h"]).isNull())
		{
		 weather.data[0].attributes.resize(++attribute_fields);
		weather.data[0].attributes[attribute_fields-1].key = "Rain mm/3 hours";
	        stringstream srain;
		srain << rain;
		weather.data[0].attributes[attribute_fields-1].value = srain.str();
		}
	 if(!(optional = root["snow"]["3h"]).isNull())
		{	 		 
		 weather.data[0].attributes.resize(++attribute_fields);
		weather.data[0].attributes[attribute_fields-1].key = "Snow mm/3 hours";
	        stringstream ssnow;
		ssnow << snow;
		weather.data[0].attributes[attribute_fields-1].value = ssnow.str();
		}
	
}

	return weather;



}




