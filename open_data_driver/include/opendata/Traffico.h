/*
 * Traffico.h
 *
 *  Created on: Feb 10, 2014
 *      Author: enrico
 */

#ifndef TRAFFICO_H_
#define TRAFFICO_H_

#include "ros/ros.h"
#include "open_data_msg/Data.h"

#include <iostream>
#include <fstream>
#include <read/read.h>
/*#include <libxml2/libxml/parser.h>
#include <libxml2/libxml/xmlreader.h>
#include <libxml2/libxml/tree.h>
#include <libxml2/libxml/xpath.h>
*/
using namespace std;

class Traffico {
public:
	Traffico();
	virtual ~Traffico();
	bool getTraffico();
	open_data_msg::Data getTrafficoFromBoundingBox(float x1, float y1,float x2, float y2);

};


#endif /* TRAFFICO_H_ */
