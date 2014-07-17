#ifndef POTENZASEGNALE_H
    #define POTENZASEGNALE_H

#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include "read/read.h"
#include "open_data_msg/Data.h"

// This is the JSON header
#include "json/json.h"

using namespace std;

    class PotenzaSegnale {
    public:
        PotenzaSegnale();
        virtual ~PotenzaSegnale();
//        bool getPotenzaSegnale(float lng, float lat);
//		bool getPotenzaSegnale(float lng,float lat, int distance, int networkType);
//		bool getPotenzaSegnale(int distance, float lng,float lat);
//		bool getPotenzaSegnale(float lng,float lat, int networkType);
		//bool getPotenzaSegnaleFromBoundingBox(float x1, float y1 , float x2 , float y2);
		
		open_data_msg::Data getPotenzaSegnale(float lng,float lat, int distance, int networkType);
		open_data_msg::Data getPotenzaSegnale(int distance, float lng,float lat);
		open_data_msg::Data getPotenzaSegnale(float lng,float lat, int networkType);
		open_data_msg::Data getPotenzaSegnale(float lng,float lat);

		open_data_msg::Data getPotenzaSegnaleFromBoundingBox(float x1, float y1 , float x2 , float y2);
		
		//bool ScriviSegnale(string s);
		open_data_msg::Data ScriviSegnale(string s);
		void parseJson(Json::Value array, open_data_msg::Data *signal);
    };

    #endif
