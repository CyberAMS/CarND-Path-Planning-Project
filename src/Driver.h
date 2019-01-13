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
#include "Map.h"
#include "Vehicle.h"
#include "Path.h"
#include "helper_functions.h"

//#include "Trajectory.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

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
const double SAFETY_DELTA_V = 2 * MPH2MS; // travel 2 mph below maximum speed
const double MAX_V = (50 * MPH2MS) - SAFETY_DELTA_V; // 50 mph minus safety delta in m/s

// longitudinal distance definitions
const double MAX_ACCELERATION_S = 10.0; // maximum total acceleration is 10 m/s^2 - lateral acceleration is treated independently here
const double MAX_WAYPOINT_DISTANCE = 10.0 * MAX_V; // don't look ahead more than 30 seconds at max speed

// lateral distance definitions
const double MAX_ACCELERATION_D = 10.0; // maximum total acceleration is 10 m/s^2 - longitudinal acceleration is treated independently here
const bool USE_FIXED_DISTANCES = true;
const vector<double> FIXED_S_DISTANCES = {0.5 * MAX_V, 1.0 * MAX_V}; // determine positions in 0.5 s and 1.0 s ahead at maximum speed

// path and trajectory parameters
const unsigned int PREVIOUS_PATH_STEPS = 10; // !!! XXX TODO Should be 3 not 30 // maximum steps considered from old trajectory
const double BACK_DISTANCE = MAX_V * SAMPLE_TIME; // spline parameter for keeping theta at segment transition (taking a large time step)

class Driver {

public:
	
	// constructor
	Driver() {}
	
	// destructor
	~Driver() {}
	
	// determine next action
	void PlanBehavior();
	
	// get map object
	Map Get_map();
	
	// get ego object
	Vehicle Get_ego();
	
	// set vehicles object vector
	void Set_vehicles(vector<Vehicle> vehicles);
	
	// get x values of path
	vector<double> Get_next_x();
	
	// get y values of path
	vector<double> Get_next_y();
	
	// get previous_path object
	Path Get_previous_path();

private:
	
	// global map
	Map map;
	
	// vehicles
	Vehicle ego; // own vehicle
	vector<Vehicle> vehicles; // other vehicles
	
	// behavior state
	//State state;
	behavior_state behavior;
	
	// main trajectory
	//Trajectory trajectory;
	vector<double> next_x_vals;
	vector<double> next_y_vals;
	
	// status values
	unsigned long time_step;
	Path previous_path;
	
	
	
	
	
	//unsigned int current_lane;
	//double current_speed;
	//unsigned int target_lane;
	//double target_speed;

};

#endif /* DRIVER_H_ */