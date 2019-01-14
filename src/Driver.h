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
#include "Trajectory.h"
#include "State.h"
#include "helper_functions.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

// general settings
//const double SAMPLE_TIME = 0.020; // 20 ms sample time of simulator (50 Hz)

// speed definitions
//const double SAFETY_DELTA_V = 2 * MPH2MS; // travel 2 mph below maximum speed
//const double MAX_V = (50 * MPH2MS) - SAFETY_DELTA_V; // 50 mph minus safety delta in m/s

// longitudinal distance definitions
//const double MAX_ACCELERATION_S = 10.0; // maximum total acceleration is 10 m/s^2 - lateral acceleration is treated independently here
//const double MAX_WAYPOINT_DISTANCE = 10.0 * MAX_V; // don't look ahead more than 30 seconds at max speed

// lateral distance definitions
//const double MAX_ACCELERATION_D = 10.0; // maximum total acceleration is 10 m/s^2 - longitudinal acceleration is treated independently here

// path and trajectory parameters
//const unsigned int PREVIOUS_PATH_STEPS = 10; // !!! XXX TODO Should be 3 not 30 // maximum steps considered from old trajectory
//const double BACK_DISTANCE = MAX_V * SAMPLE_TIME; // spline parameter for keeping theta at segment transition (taking a large time step)

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
	State state;
	
	// main trajectory
	Trajectory trajectory;
	vector<double> next_x_vals;
	vector<double> next_y_vals;
	
	// status values
	Path previous_path;
	
};

#endif /* DRIVER_H_ */