/*
 * Trasporti.h
 *
 *  Created on: Feb 10, 2014
 *      Author: enrico
 */

#ifndef TRASPORTI_H_
#define TRASPORTI_H_

#include "ros/ros.h"

//#include "open_data_msg/Coordinate.h"
//#include "open_data_msg/Polygon.h"
#include "open_data_msg/Data.h"

#include <iostream>
#include <fstream>
#include <read/read.h>
#include <csv/csv_parser.hpp>
#include <zzip/lib.h>

using namespace std;

    class Trasporti {
    public:
        Trasporti();
        virtual ~Trasporti();
        void getPolygonsFromShapeId(string shape_id,float x1, float y1 , float x2 , float y2); //dovrebbe restituirmi un vettore di coordinate(float)
        void getShapeIdFromRouteId(string route_id,float x1, float y1 , float x2 , float y2);
        void getRouteIdFromRouteType(float x1, float y1 , float x2 , float y2);

	void initialize();
	bool downloadZip(string url, string filename);
        void extractZip(string filenames[], const int num_files,string archive);

        void tripsToDb(ZZIP_FILE* file_zip);
		void shapesToDb(ZZIP_FILE* file_zip);
		void routesToDb(ZZIP_FILE* file_zip);

	void createDb();
	open_data_msg::Data getShapesFromBoundingBox(float x1, float y1, float x2, float y2);
    };




#endif /* TRASPORTI_H_ */

