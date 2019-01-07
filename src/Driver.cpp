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
#include "Car.h"
#include "Trajectory.h"
#include "Path.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

// determine next action
void Driver::plan_behavior(Car myCar, const vector<Cars> &sensor_fusion) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_DRIVER_PLAN_BEHAVIOR) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "DRIVER: plan_behavior - Start" << endl;
		cout << "  myCar: " << endl << myCar.createString();
		cout << "  sensor_fusion: " << endl << createCarsVectorString(sensor_fusion);
		
	}
	
	// define variables
	vector<unsigned int> estimated_lanes;
	
	// select follow lane behavior
	Driver::behavior = BEHAVIORS[0];
	
	// determine plan based on selected behavior
	if (Driver::behavior.name == BEHAVIORS[0].name) {
		
		// keep current lane at maximum speed
		estimated_lanes = Driver::trajectory.estimate_lanes((vector<double>){myCar.get_d()}, LANES, LANE_WIDTH);
		Driver::current_lane = estimated_lanes[0];
		Driver::current_speed = myCar.get_v();
		Driver::target_lane = Driver::current_lane;
		Driver::target_speed = MAX_V;
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_DRIVER_PLAN_BEHAVIOR) {
		
		cout << "  Driver::behavior.name: " << Driver::behavior.name << endl;
		cout << "  Driver::current_lane: " << Driver::current_lane << endl;
		cout << "  Driver::current_speed: " << Driver::current_speed << endl;
		cout << "  Driver::target_lane: " << Driver::target_lane << endl;
		cout << "  Driver::target_speed: " << Driver::target_speed << endl;
		cout << "--- DRIVER: plan_behavior - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// calculate next trajectory
void Driver::calculate_trajectory(Car myCar, Path myPreviousPath) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_DRIVER_CALCULATE_TRAJECTORY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "DRIVER: calculate_trajectory - Start" << endl;
		cout << "  myCar: " << endl << myCar.createString();
		cout << "  myPreviousPath: " << endl << myPreviousPath.createString();
		
	}
	
	// start each trajectory with current car position and short segment of previous path
	Driver::trajectory.init(myCar, myPreviousPath, PREVIOUS_PATH_STEPS, SAMPLE_TIME, Driver::maps_x, Driver::maps_y);
	
	// determine new segment based on plan
	Driver::trajectory.add(Driver::current_lane, Driver::current_speed, Driver::target_lane, Driver::target_speed, LANE_WIDTH, MAX_ACCELERATION_S, MAX_ACCELERATION_D, BACK_DISTANCE, Driver::maps_x, Driver::maps_y);
	
	// calculate xy trajectory from init and additional segment
	Driver::trajectory.calculate(myCar, BACK_DISTANCE, SAMPLE_TIME, MAX_ACCELERATION_S, Driver::maps_s, Driver::maps_x, Driver::maps_y);
	
	// return path based on trajectory
	Driver::next_x_vals = Driver::trajectory.get_x();
	Driver::next_y_vals = Driver::trajectory.get_y();
	
	// display message if required
	if (bDISPLAY && bDISPLAY_DRIVER_CALCULATE_TRAJECTORY) {
		
		cout << "  Driver::trajectory.get_s(): " << endl << createDoubleVectorString(Driver::trajectory.get_s());
		cout << "  Driver::trajectory.get_d(): " << endl << createDoubleVectorString(Driver::trajectory.get_d());
		cout << "  Driver::trajectory.get_v(): " << endl << createDoubleVectorString(Driver::trajectory.get_v());
		cout << "  Driver::next_x_vals: " << endl << createDoubleVectorString(Driver::next_x_vals);
		cout << "  Driver::next_y_vals: " << endl << createDoubleVectorString(Driver::next_y_vals);
		cout << "--- DRIVER: calculate_trajectory - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// access x values of trajectory
vector<double> Driver::get_next_x() {
	
	return Driver::next_x_vals;
	
}

// access y values of trajectory
vector<double> Driver::get_next_y() {
	
	return Driver::next_y_vals;
	
}