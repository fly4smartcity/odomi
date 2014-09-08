#include "ros/ros.h"
#include "open_data_msg/Data.h"
#include "open_data_msg/BoundingBox.h"
#include "read/read.h"
#include "csv/csv_parser.hpp"
#include "json/json.h"
#include "opendata/PotenzaSegnale.h"
#include "opendata/Trasporti.h"
#include "opendata/Traffico.h"
#include "opendata/Geoportale.h"
#include "opendata/Weather.h"
#include <sstream>


ros::Publisher open_data_pub; 

bool checkStatusOpenData(string label,int status_code, string reason)
{

		if(status_code ==1)
			{
			           ROS_INFO("%s: %s",label.c_str(),reason.c_str());
				   return true;
			}

		else if(status_code != 0)
		{
			   ROS_INFO("%s: %s",label.c_str(),reason.c_str());
			return true;
		}
		
		return false;



}


bool waitBoundingBox(open_data_msg::BoundingBox::Request  &req,
         open_data_msg::BoundingBox::Response &res)
{
Geoportale geo;
PotenzaSegnale ps;
Trasporti t;
Traffico tf;
Weather w;

bool result=true;
bool inviato=false;
open_data_msg::Data edifici,idro,signal,meteo,cavi,alberate,traffico;
//data.status.status_code = -1;
//open_data_msg::Data edifici;
	

	if(req.label == "edifici" || (req.label == "all" && req.type==0) )
	{
		ROS_INFO("Caricamento dati Edifici in corso");
		edifici = geo.getAltezzeFromBoundingBox(req.bounding_box.points[0].x,req.bounding_box.points[0].y,req.bounding_box.points[1].x,req.bounding_box.points[1].y);
	open_data_pub.publish(edifici);
	inviato=true;
		if(!checkStatusOpenData("edifici",edifici.status.status_code,edifici.status.reason))
		     res.resp = false;
		if(edifici.status.status_code == 0)
		     result = false;
	}

	 if(req.label == "idro" || (req.label == "all"&&req.type==0) )
	{
		ROS_INFO("Caricamento dati Idrografia in corso");
			idro = geo.getIdrografia(req.bounding_box.points[0].x,req.bounding_box.points[0].y,req.bounding_box.points[1].x,req.bounding_box.points[1].y);
		open_data_pub.publish(idro);
		inviato=true;
		if(!checkStatusOpenData("idro",idro.status.status_code,idro.status.reason))
		     res.resp = false;
		if(idro.status.status_code == 0)
		     result = false;
	}


	if(req.label == "signal" || (req.label == "all" && req.type==1) )
	{
		ROS_INFO("Caricamento dati Segnale LTE in corso");
		signal = ps.getPotenzaSegnaleFromBoundingBox(req.bounding_box.points[0].x,req.bounding_box.points[0].y,req.bounding_box.points[1].x,req.bounding_box.points[1].y);
	open_data_pub.publish(signal);
		inviato=true;
		if(!checkStatusOpenData("signal",signal.status.status_code,signal.status.reason))
		     res.resp = false;
		if(signal.status.status_code == 0)
		     result = false;
	}


	if(req.label == "alberate" || (req.label == "all" && req.type==0) )
	{
		ROS_INFO("Caricamento dati Zone Alberate in corso");
		alberate = geo.getAlberateFromBoundingBox(req.bounding_box.points[0].x,req.bounding_box.points[0].y,req.bounding_box.points[1].x,req.bounding_box.points[1].y);

	open_data_pub.publish(alberate);
		inviato=true;
		if(!checkStatusOpenData("alberate",alberate.status.status_code,alberate.status.reason))
		     res.resp = false;
		if(alberate.status.status_code == 0)
		     result = false;
	}


	if(req.label == "meteo" || (req.label == "all" && req.type==1) )
	{
		ROS_INFO("Caricamento dati Meteo in corso");
			meteo = w.getWeatherFromBuondingBox(req.bounding_box.points[0].x,req.bounding_box.points[0].y,req.bounding_box.points[1].x,req.bounding_box.points[1].y);

	open_data_pub.publish(meteo);
		inviato=true;
		if(!checkStatusOpenData("meteo",meteo.status.status_code,meteo.status.reason))
		     res.resp = false;
		if(meteo.status.status_code == 0)
		     result = false;
	}

	if(req.label == "traffico" || (req.label == "all" && req.type==1) )
	{
		ROS_INFO("Caricamento dati Traffico in corso");
			traffico = tf.getTrafficoFromBoundingBox(req.bounding_box.points[0].x,req.bounding_box.points[0].y,req.bounding_box.points[1].x,req.bounding_box.points[1].y);

	open_data_pub.publish(traffico);
		inviato=true;
		if(!checkStatusOpenData("traffico",traffico.status.status_code,traffico.status.reason))
		     res.resp = false;
		if(traffico.status.status_code == 0)
		     result = false;

	}

	if(req.label == "linee_aeree" || (req.label == "all" && req.type==0) )
	{
		ROS_INFO("Caricamento dati Linee aeree in corso");
		cavi = t.getShapesFromBoundingBox(req.bounding_box.points[0].x,req.bounding_box.points[0].y,req.bounding_box.points[1].x,req.bounding_box.points[1].y);
	open_data_pub.publish(cavi);
	inviato=true;
		if(!checkStatusOpenData("linee_aeree",cavi.status.status_code,cavi.status.reason))
		     res.resp = false;
		if(cavi.status.status_code == 0)
		     result = false;
	}

		/*if(data.status.status_code ==0)
			{
				   open_data_pub.publish(data);
			           ROS_INFO("%s",data.status.reason.c_str());
				   res.resp = true;
				   return true;
			}

		else if(data.status.status_code != -1)
		{
			   open_data_pub.publish(data);	
			   ROS_INFO("%s",data.status.reason.c_str());
			  res.resp = false;
			return true;
		}
			
                           res.resp = false;
			   return false;*/
	if(inviato)
		ROS_INFO("Invio OPEN DATA completato.");

	else
	{
		ROS_INFO("Invio dati non avvenuto. Controllare i parametri del client.");
		return false;
	}

if(result)
	{
	 res.resp=true;
	}
	return result;
}


 

int main(int argc, char **argv)
{
 ros::init(argc, argv, "open_data_driver"); 
 ros::NodeHandle n;

 open_data_pub = n.advertise<open_data_msg::Data>("opendata", 10000);
ros::ServiceServer service = n.advertiseService("bounding_box", waitBoundingBox);

ros::spin();

  return 0;
}




