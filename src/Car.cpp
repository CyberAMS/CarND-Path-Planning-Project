/*
 * Car.cpp
 *
 * Car class
 *
 * Created on 01/03/2019
 * Author: Andre Strobel
 *
 */

#include <iostream>
#include <string>
#include <vector>
#include "helper_functions.h"
#include "Car.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

// set state of car
void Car::set_state(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed) {
	
	Car::car_x = car_x;
	Car::car_y = car_y;
	Car::car_s = car_s;
	Car::car_d = car_d;
	Car::car_yaw = car_yaw;
	Car::car_speed = car_speed;
	
}