/*
 * Driver.h
 *
 * Driver class
 *
 * Created on 01/03/2019
 * Author: Andre Strobel
 *
 */

#ifndef DRIVER_H_
#define DRIVER_H_

#include <iostream>
#include <string>
#include <vector>
#include "helper_functions.h"
#include "Car.h"
#include "Trajectory.h"
#include "Path.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

struct Cars {
	
	unsigned int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
	
};

struct behavior_state {
	
	string name;
	vector<string> next_states;
	
};

// general settings
const double SAMPLE_TIME = 0.020; // 20 ms sample time (50 Hz)

// possible behavior states and transition options
// DS:   drive straight
// FL:   follow lane at constant speed
// KL:   keep lane and follow vehicle in front
// PLCL: prepare lane change to left
// PLCR: prepare lane change to right
// LCL:  lane change to left
// LCR:  lane change to right
const vector<behavior_state> BEHAVIORS 
	{{.name = "DS", .next_states = {"DS"}},
	 {.name = "FL", .next_states = {"FL"}},
	 {.name = "KL", .next_states = {"KL", "PLCL", "PLCR"}},
	 {.name = "PLCL", .next_states = {"KL", "PLCL", "LCL"}},
	 {.name = "PLCR", .next_states = {"KL", "PLCR", "LCR"}},
	 {.name = "LCL", .next_states = {"KL"}},
	 {.name = "LCR", .next_states = {"KL"}}};

// longitudinal distance definitions
const double ZERO_S = 0;
const double MAX_S = 10; // define path with end point in 10 m ahead
const double MAX_ACCELERATION_S = 10; // maximum total acceleration is 10 m/s^2 - lateral acceleration is neglected

// lateral distance definitions
const double LANE_WIDTH = 4;
const unsigned int LANE_1 = 1;
const unsigned int LANE_2 = 2;
const unsigned int LANE_3 = 3;

// speed definitions
const double ZERO_V = 0;
const double MAX_V = 22.352; // 50 mph in m/s

// straight trajectory
const vector<double> STRAIGHT_S = {ZERO_S, MAX_S};
const vector<double> STRAIGHT_CONSTANT_MAX_SPEED_V = {MAX_V, MAX_V};
const vector<double> STRAIGHT_ZERO_TO_MAX_SPEED_V = {ZERO_V, MAX_V};

class Driver {

public:
	
	// constructor
	Driver() {}
	
	// constructor
	Driver(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
		
		// initialize map information
		Driver::maps_s = maps_s;
		Driver::maps_x = maps_x;
		Driver::maps_y = maps_y;
		
		// start with straight driving
		Driver::behavior = BEHAVIORS[0];
		
		// start in lane 1
		Driver::lane = LANE_1;
		
	}
	
	// destructor
	~Driver() {}
	
	// access x values of path
	vector<double> get_next_x();
	
	// access y values of path
	vector<double> get_next_y();
	
	// determine next action
	void plan_behavior(Car myCar, Path myPreviousPath, const vector<Cars> &sensor_fusion);
	
	// calculate next trajectory
	void calculate_trajectory(Car myCar);

private:
	
	// map
	vector<double> maps_s;
	vector<double> maps_x;
	vector<double> maps_y;
	
	// trajectory
	Trajectory trajectory;
	
	// lane
	unsigned int lane;
	
	// path values
	vector<double> next_x_vals;
	vector<double> next_y_vals;
	
	// behavior
	behavior_state behavior;

};

#endif /* DRIVER_H_ */