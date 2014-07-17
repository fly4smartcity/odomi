/*
 * Geoportale.h
 *
 *  Created on: Feb 14, 2014
 *      Author: enrico
 */

#ifndef GEOPORTALE_H_
#define GEOPORTALE_H_

#include "ros/ros.h"

//#include "open_data_msg/Coordinate.h"
//#include "open_data_msg/Polygon.h"
#include "open_data_msg/Data.h"

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <read/read.h>
#include <ogrsf_frmts.h>
#include <json/json.h>
#include "geometry_msgs/Polygon.h"


class Geoportale {
public:
	Geoportale();
	virtual ~Geoportale();
	void getData();
	open_data_msg::Data getAltezzeFromBoundingBox(float x1, float y1 , float x2,float y2);



	open_data_msg::Data getAlberateFromBoundingBox(float x1, float y1 , float x2,float y2);
	open_data_msg::Data getAreeVerdi(float x1, float y1 , float x2,float y2);
	open_data_msg::Data getIdrografia(float x1, float y1 , float x2,float y2);
	open_data_msg::Data getPortici(float x1, float y1 , float x2,float y2);
	open_data_msg::Data getPonti(float x1, float y1 , float x2,float y2);

private:
	Read rd;
	string getXml(string url);
	bool writeXml(string xml,const string filename);

	open_data_msg::Data getCoordinates(const string filein, string label ,string attribute_name[],int numAttr);
	void parseGeometry(Json::Value coordinates, geometry_msgs::Polygon *area, bool b);




};

#endif /* GEOPORTALE_H_ */
