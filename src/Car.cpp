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
using std::to_string;
using std::cout;
using std::endl;

// set state of car
void Car::set_state(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed) {
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "CAR: set_state - Start" << endl;
		cout << "  car_x: " << car_x << endl;
		cout << "  car_y: " << car_y << endl;
		cout << "  car_s: " << car_s << endl;
		cout << "  car_d: " << car_d << endl;
		cout << "  car_yaw: " << car_yaw << endl;
		cout << "  car_speed: " << car_speed << endl;
		
	}
	
	Car::car_x = car_x;
	Car::car_y = car_y;
	Car::car_s = car_s;
	Car::car_d = car_d;
	Car::car_yaw = car_yaw;
	Car::car_speed = car_speed;
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "--- CAR: set_state - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// get car_s value
double Car::get_s() {
	
	return Car::car_s;
	
}

// get car_d value
double Car::get_d() {
	
	return Car::car_d;
	
}

// get car_v value
double Car::get_v() {
	
	return Car::car_speed;
	
}

// display Car object as string
string Car::createString() {
	
	//define variables
	string text = "";
	
	// add information about car to string
	text += DISPLAY_PREFIX;
	text += "car_x=" + to_string(Car::car_x) + " ";
	text += "car_y=" + to_string(Car::car_y) + " ";
	text += "car_s=" + to_string(Car::car_s) + " ";
	text += "car_d=" + to_string(Car::car_d) + " ";
	text += "car_yaw=" + to_string(Car::car_yaw) + " ";
	text += "car_speed=" + to_string(Car::car_speed) + "\n";
	
	// return output
	return text;
	
}