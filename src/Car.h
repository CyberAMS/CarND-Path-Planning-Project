/*
 * Car.h
 *
 * Car class
 *
 * Created on 01/03/2019
 * Author: Andre Strobel
 *
 */

#ifndef CAR_H_
#define CAR_H_

#include <iostream>
#include <string>
#include <vector>
#include "helper_functions.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

class Car {

public:
	
	// constructor
	Car() {}
	
	// destructor
	~Car() {}
	
	// set state of car
	void set_state(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed);

private:
	
	// state values
	double car_x;
	double car_y;
	double car_s;
	double car_d;
	double car_yaw;
	double car_speed;

};

#endif /* CAR_H_ */