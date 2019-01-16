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

class Driver {

public:
	
	// constructor
	Driver() {}
	
	// destructor
	~Driver() {}
	
	// determine next action
	void PlanBehavior();
	
	// get map object
	Map* Get_map();
	
	// get ego object
	Vehicle* Get_ego();
	
	// set vehicles object vector
	void Set_vehicles(vector<Vehicle> vehicles);
	
	// get x values of path
	vector<double> Get_next_x();
	
	// get y values of path
	vector<double> Get_next_y();
	
	// get previous_path object
	Path* Get_previous_path();

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