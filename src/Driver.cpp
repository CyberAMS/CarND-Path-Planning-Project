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
#include "helper_functions.h"
#include "Driver.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

// access x values of trajectory
vector<double> Driver::get_next_x() {
	
	return Driver::next_x_vals;
	
}

// access y values of trajectory
vector<double> Driver::get_next_y() {
	
	return Driver::next_y_vals;
	
}

// determine next action
void Driver::plan_behavior(const Car &myCar, const Path &myPreviousPath, const vector<Cars> &sensor_fusion) {
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "DRIVER: plan_behavior - Start" << endl;
		
	}
	
	// select follow lane behavior
	Driver::behavior = BEHAVIORS[1];
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "  Driver::behavior.name: " << Driver::behavior.name << endl;
		cout << "--- DRIVER: plan_behavior - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// calculate next trajectory
void Driver::calculate_trajectory() {
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "DRIVER: calculate_trajectory - Start" << endl;
		
	}
	
	// define variables
	vector<vector<double>> path_xy;
	
	// determine trajectories
	if (Driver::behavior.name == BEHAVIORS[0].name) {
		// determine trajectory for straight driving
		
		// directly set xy parameters - TODO !!!!!!!!!!!!!
		
	} else if (Driver::behavior.name == BEHAVIORS[1].name) {
		// determine trajectory for keep lane
		
		Driver::trajectory.set(STRAIGHT_S, Driver::trajectory.calculate_d_from_lane((vector<unsigned int>){Driver::lane, Driver::lane}, LANE_WIDTH), STRAIGHT_CONSTANT_MAX_SPEED_V, MAX_ACCELERATION_S, SAMPLE_TIME);
		
	} else {
		// determine trajectory if behavior not defined
		
		Driver::trajectory.set(STRAIGHT_S, Driver::trajectory.calculate_d_from_lane((vector<unsigned int>){Driver::lane, Driver::lane}, LANE_WIDTH), STRAIGHT_CONSTANT_MAX_SPEED_V, MAX_ACCELERATION_S, SAMPLE_TIME);
		
	};
	
	// return path based on trajectory
	path_xy = Driver::trajectory.get_xy(Driver::trajectory.get_s(), Driver::trajectory.get_d(), Driver::maps_s, Driver::maps_x, Driver::maps_y);
	Driver::next_x_vals = path_xy[0];
	Driver::next_y_vals = path_xy[1];
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "  Driver::trajectory.get_s(): " << endl << createDoubleVectorString(Driver::trajectory.get_s());
		cout << "  Driver::trajectory.get_d(): " << endl << createDoubleVectorString(Driver::trajectory.get_d());
		cout << "  Driver::trajectory.get_v(): " << endl << createDoubleVectorString(Driver::trajectory.get_v());
		cout << "  Driver::next_x_vals: " << endl << createDoubleVectorString(Driver::next_x_vals);
		cout << "  Driver::next_y_vals: " << endl << createDoubleVectorString(Driver::next_y_vals);
		cout << "--- DRIVER: calculate_trajectory - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}