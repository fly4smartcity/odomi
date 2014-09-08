
#include "open_data_msg/Data.h"
#include "open_data_msg/Open_data.h"
#include <cmath>
#include <opendata/Traffico.h>
#include <string>
#include <iostream>
#include <sstream>
#include "xml/pugiconfig.hpp"
#include "xml/pugixml.hpp"
#include <read/read.h>

Traffico::Traffico() {}

Traffico::~Traffico(){}

bool Traffico::getTraffico(){
	Read rd;
	string url = "http://opendata.5t.torino.it/get_fdt";
	string s = rd.read(url);
/*	ofstream of("traffico.xml");
	if(!of)
	{
		return false;

	}

	of << s;
*/
	float lat,lng,flow;
	 pugi::xml_document doc;

	 pugi::xml_parse_result result = doc.load(s.c_str());

	 //std::cout << "Load result: " << result.description() << ", mesh name: " << doc.child("mesh").attribute("name").value() << std::endl;
	 pugi::xml_node root = doc.child("traffic_data");



	     for (pugi::xml_node elem = root.first_child(); elem; elem = elem.next_sibling())
	     {
	    	 string nome = elem.name();
	    	 if(nome == "FDT_data")
	         {

	    		 lat = elem.attribute("lat").as_float();
	    		 lng = elem.attribute("lng").as_float();


				 pugi::xml_node speedflow = elem.first_child();
				 flow = speedflow.attribute("flow").as_float();



	         }
	     }

	return true;
}

open_data_msg::Data Traffico::getTrafficoFromBoundingBox(float x1,float y1,float x2,float y2){

	Read rd;

	open_data_msg::Data vettore;			    
	open_data_msg::Open_data op;

	string label = "traffico";
	string key = "flusso di traffico";
	int numAttr=1;

	string url = "http://opendata.5t.torino.it/get_fdt";
	string s = rd.read(url);
	
	if(s=="")
	{
	vettore.status.status_code = -2;
        vettore.status.reason = "Connessione al server non riuscita";
	return vettore;
	}

/*	ofstream of("traffico.xml");
	if(!of)
	{
		return false;

	}

	of << s;
*/
	float latA,lngA,latB,lngB,latA_rad,lngA_rad,latB_rad,lngB_rad,teta;
	float offset;

//ordino le coordinate in modo che x1 < x2 e che y1 < y2
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


	 pugi::xml_document doc;

//pugi::xml_parse_result result = doc.load(s.c_str());
doc.load(s.c_str());
	 //std::cout << "Load result: " << result.description() << ", mesh name: " << doc.child("mesh").attribute("name").value() << std::endl;
	 pugi::xml_node root = doc.child("traffic_data");

	int incroci=0;
	bool result = false;
	

	     for (pugi::xml_node elem = root.first_child(); elem; elem = elem.next_sibling())
	     {
	    	 string nome = elem.name();
	    	 if(nome == "FDT_data")
	         {


	    		 offset = elem.attribute("offset").as_float();
	    		 latA = elem.attribute("lat").as_float();
	    		 lngA = elem.attribute("lng").as_float();
			//cout << "A: " << latA << " " << lngA << endl;

			//calcola distanza tra 2 punti date le loro longitudini e latitudini

				const float R = 6372795.477598;//raggio terrestre in m

				 //trasformo in radianti
/*				 latA_rad=(latA*M_PI)/180;
				 lngA_rad=(lngA*M_PI)/180;


string direction = elem.attribute("direction").as_string();

if(direction=="negative")
	teta=M_PI;
else if(direction=="positive")
	teta=-M_PI;
else
	cout << "direction not found"<<endl;


//teta = M_PI;
float k = offset/R;

latB_rad =asin(sin(latA_rad)*cos(k) + cos(latA_rad) * sin(k) * cos(teta));
lngB_rad = lngA_rad + atan2(sin(teta)*sin(k) *cos(latA_rad) , cos(k) - (sin(latA_rad) * sin(latB_rad)));


//ritrasformo in gradi
	
latB=(latB_rad*180)/M_PI;
lngB=(lngB_rad*180)/M_PI;


//cout <<"B: " << latB << " " <<lngB <<endl;				 
*/
	    		 if( (latA < y2 && latA > y1 ) && (lngA > x1  && lngA < x2))
			   {
				 pugi::xml_node speedflow = elem.first_child();

					result=true;
					vettore.data.resize(incroci+1);
				    	 vettore.data[incroci].type = op.TYPE_DYNAMIC;
		    			 vettore.data[incroci].label.assign(label);

				vettore.data[incroci].attributes.resize(numAttr);
				 vettore.data[incroci].attributes[0].key.assign(key);
				 vettore.data[incroci].attributes[0].value = speedflow.attribute("flow").as_string();				
				vettore.data[incroci].area.points.resize(1);

				vettore.data[incroci].area.points[0].y = (latA);
				vettore.data[incroci].area.points[0].x = (lngA);
				vettore.data[incroci].area.points[0].z = 0.0;
/*				vettore.data[incroci].area.points[1].y = (latB);
				vettore.data[incroci].area.points[1].x = (lngB);
				vettore.data[incroci].area.points[1].z = 0.0;

*/

				incroci++;
			  }
	         }
	     }

	if(result)
	{	
	     vettore.status.status_code = 1;
	     vettore.status.reason = "OK";
	}
	else
	{
			  vettore.status.status_code = -1;
			  vettore.status.reason = "Nessun Risultato";

	}			

	return vettore;
}



