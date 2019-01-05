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

using std::vector;
using std::string;

// access x values of trajectory
vector<double> Driver::get_next_x() {
	
	return Driver::next_x_vals;
	
}

// access y values of trajectory
vector<double> Driver::get_next_y() {
	
	return Driver::next_y_vals;
	
}

// determine next action
void Driver::plan_behavior(Car myCar, Path myPreviousPath, vector<Cars> sensor_fusion) {
	
	// select straight driving
	Driver::behavior = BEHAVIORS[0];
	
}

// calculate next trajectory
void Driver::calculate_trajectory() {
	
	// define variables
	vector<vector<double>> trajectory_xy;
	
	if (Driver::behavior.name == BEHAVIORS[0].name) {
		// determine trajectory for straight driving
		
		Driver::trajectory.set(STRAIGHT_S, Driver::trajectory.get_d_from_lane(STRAIGHT_LANE_1, LANE_WIDTH), STRAIGHT_CONSTANT_MAX_SPEED_V);
		
	} else {
		// determine trajectory if behavior not defined
		
		Driver::trajectory.set(STRAIGHT_S, Driver::trajectory.get_d_from_lane(STRAIGHT_LANE_1, LANE_WIDTH), STRAIGHT_CONSTANT_MAX_SPEED_V);
		
	};
	
	trajectory_xy = Driver::trajectory.get_xy(Driver::trajectory.get_s(), Driver::trajectory.get_d(), Driver::maps_s, Driver::maps_x, Driver::maps_y);
	Driver::next_x_vals = trajectory_xy[0];
	Driver::next_y_vals = trajectory_xy[1];
	
}