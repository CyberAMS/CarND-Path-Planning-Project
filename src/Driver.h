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
const double SAMPLE_TIME = 0.020; // 20 ms sample time of simulator (50 Hz)

// possible behavior states and transition options
// FL:   follow lane at maximum speed
// KL:   keep lane and follow vehicle in front
// PLCL: prepare lane change to left
// PLCR: prepare lane change to right
// LCL:  lane change to left
// LCR:  lane change to right
const vector<behavior_state> BEHAVIORS 
	{{.name = "FL", .next_states = {"FL"}},
	 {.name = "KL", .next_states = {"KL", "PLCL", "PLCR"}},
	 {.name = "PLCL", .next_states = {"KL", "PLCL", "LCL"}},
	 {.name = "PLCR", .next_states = {"KL", "PLCR", "LCR"}},
	 {.name = "LCL", .next_states = {"KL"}},
	 {.name = "LCR", .next_states = {"KL"}}};

// speed definitions
const double SAFETY_DELTA_V = 0.89408; // travel 2 mph below maximum speed
const double MAX_V = 22.352 - SAFETY_DELTA_V; // 50 mph minus safety delta in m/s

// longitudinal distance definitions
const double MAX_ACCELERATION_S = 10.0; // maximum total acceleration is 10 m/s^2 - lateral acceleration is treated independently here
const double MAX_WAYPOINT_DISTANCE = 30.0 * MAX_V; // don't look ahead more than 30 seconds at max speed

// lateral distance definitions
const double LANE_WIDTH = 4.0;
const unsigned int LANE_1 = 1;
const unsigned int LANE_2 = 2;
const unsigned int LANE_3 = 3;
const vector<unsigned int> LANES = {LANE_1, LANE_2, LANE_3};
const double MAX_ACCELERATION_D = 10.0; // maximum total acceleration is 10 m/s^2 - longitudinal acceleration is treated independently here

// path and trajectory parameters
const unsigned int PREVIOUS_PATH_STEPS = 3; // maximum steps considered from old trajectory
const double BACK_DISTANCE = MAX_V * SAMPLE_TIME; // spline parameter for keeping theta at segment transition (taking a large time step)

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
		
		// start with following the lane
		Driver::behavior = BEHAVIORS[0];
		
	}
	
	// destructor
	~Driver() {}
	
	// determine next action
	void plan_behavior(Car myCar, const vector<Cars> &sensor_fusion);
	
	// calculate next trajectory
	void calculate_trajectory(Car myCar, Path myPreviousPath);
	
	// access x values of path
	vector<double> get_next_x();
	
	// access y values of path
	vector<double> get_next_y();

private:
	
	// map
	vector<double> maps_s;
	vector<double> maps_x;
	vector<double> maps_y;
	
	// plan
	behavior_state behavior;
	unsigned int current_lane;
	double current_speed;
	unsigned int target_lane;
	double target_speed;
	
	// trajectory
	Trajectory trajectory;
	
	// path values
	vector<double> next_x_vals;
	vector<double> next_y_vals;

};

#endif /* DRIVER_H_ */