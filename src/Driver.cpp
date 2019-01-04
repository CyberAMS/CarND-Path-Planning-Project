/*
 * Driver.cpp
 *
 * Driver class
 *
 * Created on 01/03/2019
 * Author: Andre Strobel
 *
 */

#include <iostream>
#include <string>
#include <vector>
#include "Driver.h"

using namespace std;

// access x values of trajectory
std::vector<double> Driver::get_next_x() {
	
	return Driver::next_x_vals;
	
}

// access y values of trajectory
std::vector<double> Driver::get_next_y() {
	
	return Driver::next_y_vals;
	
}

// determine next action
void Driver::plan_behavior(Car myCar, Path myPreviousPath, std::vector<Cars> sensor_fusion) {
	
	
	
}

// calculate next trajectory
void Driver::calculate_trajectory() {
	
	
	
}