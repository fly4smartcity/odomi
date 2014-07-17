/*
 * Geoportale.cpp
 *
 *  Created on: Feb 14, 2014
 *      Author: enrico
 */

#include <climits>
#include <opendata/Geoportale.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include "read/read.h"
#include "cpl_port.h"
#include <unistd.h>
#include <sys/stat.h>

//#include "open_data_msg/Coordinate.h"
////#include "open_data_msg/Polygon.h"
#include "geometry_msgs/Polygon.h"
#include "open_data_msg/Data.h"
#include "open_data_msg/Open_data.h"
//#include <opendata/Oggetto.h>



Geoportale::Geoportale() {VSIInstallCurlStreamingFileHandler();  	}

Geoportale::~Geoportale() {}

/*open_data_msg::Data Geoportale::getData(float x1, float y1 , float x2,float y2)
{
getAltezzeFromBoundingBox(x1,y1,x2,y2);
getAlberateFromBoundingBox(x1,y1,x2,y2);
getPontiFromBoundingBox(x1,y1,x2,y2);
getPorticiFromBoundingBox(x1,y1,x2,y2);
getAreeVerdiFromBoundingBox(x1,y1,x2,y2);
getIdrografiaFromBoundingBox(x1,y1,x2,y2);
}*/

open_data_msg::Data Geoportale::getAltezzeFromBoundingBox(float x1, float y1 , float x2,float y2)
{
	string rxml ="http://geomap.reteunitaria.piemonte.it/ws/siccms/coto-01/wmsg01/wms_sicc31_carta_tecnica_bn?service=wfs&version=1.1.0&request=GetFeature&TypeName=namespace:CartaTecnica&propertyname=altezza_edificio,msGeometry&SRSNAME=EPSG:4326&bbox=";
	string filename = "altezze.xml";
	stringstream sstm;
	sstm << rxml <<"" << x1 << "," << y1 << "," << x2 << "," << y2;
	string url = sstm.str();

	string attribute[1] =  "altezza_edificio";

	writeXml(getXml(url),filename);		
	return getCoordinates(filename,"edifici",attribute,1);

		
//return getCoordinates(url,"edifici",attribute,1);

}



open_data_msg::Data Geoportale::getAlberateFromBoundingBox(float x1, float y1 , float x2,float y2)
{
	string rxml ="http://geomap.reteunitaria.piemonte.it/ws/siccms/coto-01/wmsg01/wms_sicc11_alberate?service=wfs&version=1.1.0&request=GetFeature&TypeName=AlberatePt&propertyname=altezza,msGeometry&SRSNAME=EPSG:4326&bbox=";
	string filename = "alberate.xml";
	stringstream sstm;
	sstm << rxml <<"" << x1 << "," << y1 << "," << x2 << "," << y2  ;
	string url = sstm.str();

	string attribute[1] =  "altezza";

	writeXml(getXml(url),filename);
	return getCoordinates(filename,"alberate",attribute,1);
	
//return getCoordinates(url,"alberate",attribute,1);
}

open_data_msg::Data Geoportale::getAreeVerdi(float x1, float y1 , float x2,float y2)
{
	string rxml ="geomap.reteunitaria.piemonte.it/ws/siccms/coto-01/wmsg01/wms_sicc12_aree_verdi?service=WFS&version=1.1.0&request=getFeature&typename=AreeVerdiPl&SRSNAME=EPSG:4326&bbox=";
		string filename = "aree_verdi.xml";
		stringstream sstm;
			sstm << rxml <<"" << x1 << "," << y1 << "," << x2 << "," << y2  ;

			string url = sstm.str();
	string attribute[1] =  "";

		writeXml(getXml(url),filename);	
		return getCoordinates(filename,"aree_verdi",attribute,1);

//return getCoordinates(url,"aree_verdi",attribute,1);
}

open_data_msg::Data Geoportale::getIdrografia(float x1, float y1 , float x2,float y2)
{
	string rxml =/*"geomap.reteunitaria.piemonte.it/ws/taims/rp-01/taimsidrowms/wms_idro?service=WFS&version=1.1.0&request=GetFeature&typename=IdrografiaLineare100,IdrografiaLineare10,IdrografiaAreale,Laghi,Canali&SRSNAME=EPSG:4326&bbox=";*/
"http://geomap.reteunitaria.piemonte.it/ws/taims/rp-01/taimsidrowms/wms_idro?service=WFS&version=1.1.0&request=GetFeature&typename=IdrografiaLineare100&SRSNAME=EPSG:4326&bbox=";
		string filename = "idro.xml";
		stringstream sstm;
			sstm << rxml  <<"" << x1 << "," << y1 << "," << x2 << "," << y2 ;

			string url = sstm.str();
	string attribute[1] =  "";

		writeXml(getXml(url),filename);
	return getCoordinates(filename,"idro",attribute,1);
//	return getCoordinates(url,"idro",attribute,1);
}

open_data_msg::Data Geoportale::getPortici(float x1, float y1 , float x2,float y2)
{
	string rxml ="http://geomap.reteunitaria.piemonte.it/ws/siccms/coto-01/wmsg01/wms_sicc31_carta_tecnica_bn?service=wfs&version=1.1.0&request=GetFeature&TypeName=namespace:PorticiEcc&SRSNAME=EPSG:4326&bbox=";
		string filename = "portici.xml";
		stringstream sstm;
		sstm << rxml <<"" << x1 << "," << y1 << ",=" << x2 << "," << y2  ;
		string url = sstm.str();
	
	string attribute[1] =  "";

			writeXml(getXml(url),filename);
	

		return getCoordinates(url,"portici",attribute,1);


}

open_data_msg::Data Geoportale::getPonti(float x1, float y1 , float x2,float y2)
{
	string rxml ="http://geomap.reteunitaria.piemonte.it/ws/siccms/coto-01/wmsg01/wms_sicc31_carta_tecnica_bn?service=wfs&version=1.1.0&request=GetFeature&TypeName=namespace:OpereInfrastruttura&SRSNAME=EPSG:4326&bbox=";
		string filename = "ponti.xml";
		stringstream sstm;
			sstm << rxml <<"" << x1 << "," << y1 << "," << x2 << "," << y2  ;
			string url = sstm.str();

			string attribute[1] =  "";

	writeXml(getXml(url),filename);
	return getCoordinates(filename,"portici",attribute,1);
	

//		return getCoordinates(url,"ponti",attribute,1);
}




string Geoportale::getXml(string url)
{
string xml = rd.read(url);
return xml;
}


bool Geoportale::writeXml(string xml,const string filename)
{
		FILE *F = fopen(filename.c_str(),"w");

	if(!F)
	{
        cout<<"Errore nella creazione del file: " << filename;
        return false;
    	}


fprintf(F,"%s",xml.c_str());

fclose(F);

return true;
}




open_data_msg::Data Geoportale::getCoordinates(const string filein, string label ,string attribute_name[],int numAttr)
{
 OGRRegisterAll();

OGRDataSource  *poDS;
open_data_msg::Data vettore;
open_data_msg::Open_data op;
int featureNum=0;  


/*stringstream sstm;
	sstm << "/vsicurl/" << filein;
	string url = sstm.str();
*/



//poDS = OGRSFDriverRegistrar::Open(url.c_str());
poDS = OGRSFDriverRegistrar::Open(filein.c_str());

  if( poDS == NULL )
  {
   //   printf( "Open of %s failed.\n",filein.c_str());
      vettore.status.status_code = -2;
      vettore.status.reason = "Connessione al server non riuscita";

//	vettore.status.status_code = -1;
//  	vettore.status.reason = "Nessun Risultato o Connessione al server non riuscita";

      return vettore;
  }



//  int nLayerCount = poDS->GetLayerCount();  //da come risultato sempre 1, solo un livello
  int iLayer = 0; //mi posiziono sul primo livello
               OGRLayer   *poLayer = poDS->GetLayer(iLayer);
               OGRFeature *poFeature;
               OGRGeometry *poGeometry;
   

//unsigned int numFeatures = poLayer->GetFeatureCount(iLayer);
// <<"features: " << numFeatures;
//alloco vettore open data, tanti quanti sono le features		
//vettore.data.resize(numFeatures);		

     bool bFoundFeature;
     bool result= false;

     do
     {
         bFoundFeature = FALSE;

             while((poFeature = poLayer->GetNextFeature()) != NULL)
             {
		result=true;
		vettore.data.resize(featureNum+1);
            	 vettore.data[featureNum].type = op.TYPE_STATIC;
            	 vettore.data[featureNum].label.assign(label);
            	  vettore.data[featureNum].attributes.resize(numAttr);

//            	  vettore.data[featureNum].attributes.value.assign("10");
  //          	  vettore.data[featureNum].attributes.key.assign("altezza");


                 bFoundFeature = TRUE;
                 if(attribute_name[0]!= "")
                 {
                         for( int iField = 0; iField < poFeature->GetFieldCount(); iField++ )
                         {

                             OGRFieldDefn    *poFDefn = poFeature->GetFieldDefnRef(iField);
                             string nomecampo = poFDefn->GetNameRef();
                             bool flag = false;

                           for(int i=0; i < numAttr && !flag;i++)
                            {
		                     if(nomecampo == attribute_name[i])
		                     {
		                    	 //campo trovato, non ha senso proseguire
		                    	 flag=true;

		                         vettore.data[featureNum].attributes[i].key.assign(nomecampo);


				             if( poFeature->IsFieldSet( iField ) )
				                      	  vettore.data[featureNum].attributes[i].value.assign(poFeature->GetFieldAsString( iField ));

				             else
				                      	  vettore.data[featureNum].attributes[i].value.assign("-1");
		                     }

                            }//chiude for che scorre il vettore di stringhe attribute_name[]
			}//chiude for
                     }//chiude if attribute_name

                         poGeometry = poFeature->GetGeometryRef();

                         int dimGeom = poGeometry->getDimension();

                          char* pszJson = poGeometry->exportToJson();
//			  printf("%s ",pszJson);
                          //Let's parse it
                          Json::Value root;
                          Json::Reader reader;
                          bool parsedSuccess = reader.parse(pszJson,
                                                            root,
                                                            false);
                          if(!parsedSuccess)
                          {
                            // Report failures and their locations
                            // in the document.
                            cout<<"Failed to parse JSON"<<endl
                                <<reader.getFormatedErrorMessages()
                                <<endl;
                            	exit(1);
                          }

                          // Let's extract the array contained
                          // in the root object
                          Json::Value coordinates = root["coordinates"];
			  


                          // Iterate over sequence elements and
			  int j=0;
	                  parseGeometry(coordinates,&vettore.data[featureNum].area,true);


                 OGRFeature::DestroyFeature(poFeature);



                 featureNum++;


             }//chiude while GetNextFeature

     } while (bFoundFeature);

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



void Geoportale::parseGeometry(Json::Value coordinates,geometry_msgs::Polygon *area, bool b)
{

	Json::Value child;
	static int j;
	
//se passo b=true, allora devo riazzerare j
if(b)
 j=0;


    for( int i=0; i<coordinates.size(); i++)
    {


              	  child = coordinates[i];

              	  if(child.isArray())//controllo se contiene ancora un array al suo interno
              		  {
              		  parseGeometry(child,area,false);
              		  }

              	  else if(i%2==0)
              		  {
				area->points.resize(j+1);
				area->points[j].x=coordinates[i].asDouble();
              			area->points[j].z=0.0;
              		  }
              		  else /*if(i%2==1)*/
              		  {
              			area->points[j].y = coordinates[i].asDouble();
              		  }

    }
    

j++;


}




