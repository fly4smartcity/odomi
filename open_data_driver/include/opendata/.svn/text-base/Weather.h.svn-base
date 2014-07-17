/*
 * Weather.h
 *
 *  Created on: May 10, 2014
 *      Author: enrico
 */

#ifndef WEATHER_H_
#define WEATHER_H_

#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <read/read.h>
#include "open_data_msg/Data.h"

// This is the JSON header
#include "json/json.h"


class Weather {
public:
	Weather();			
	open_data_msg::Data getWeatherFromBuondingBox(float x1, float y1, float x2, float y2);
	open_data_msg::Data ScriviWeather(string url);
	virtual ~Weather();
};

#endif /* WEATHER_H_ */

