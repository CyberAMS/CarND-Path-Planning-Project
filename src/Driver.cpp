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

// determine next action
void Driver::PlanBehavior() {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_DRIVER_PLANBEHAVIOR) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "DRIVER: PlanBehavior - Start" << endl;
		
	}
	
	// define variables
	unsigned long from_step = 0;
	unsigned long finished_steps = 0;
	vector<behavior_state> next_possible_behaviors;
	unsigned int count = 0;
	Trajectory next_possible_trajectory;
	double cost;
	double minimal_cost = std::numeric_limits<double>::max();
	behavior_state best_behavior = this->state.behavior;
	Trajectory best_trajectory = this->trajectory;
	
	// initialize all objects for next step
	if (this->trajectory.Get_is_initialized()) {
		
		from_step = NUM_PREVIOUS_PATH_STEPS;
		
	} else {
		
		from_step = NO_PREVIOUS_PATH_STEPS;
		
	}
	finished_steps = this->trajectory.Init(this->ego, this->previous_path);
	this->state.Init(this->ego, this->trajectory, finished_steps);
	
	// get next possible states
	next_possible_behaviors = this->state.GetNextPossibleBehaviors(this->Get_ego()->Get_lane());
	
	// generate trajectories for all states
	for (count = 0; count < next_possible_behaviors.size(); count++) {
		
		// generate trajectory for current state
		next_possible_trajectory = this->state.GenerateTrajectoryFromBehavior(this->map, this->ego, this->trajectory, from_step, next_possible_behaviors[count]);
		
		// check whether trajectory is valid
		if (next_possible_trajectory.Valid()) {
			
			// determine cost of trajectory
			cost = next_possible_trajectory.Cost();
			
			// found better trajectory
			if (cost < minimal_cost) {
				
				// save lowest cost trajectory
				minimal_cost = cost;
				best_behavior = next_possible_behaviors[count];
				best_trajectory = next_possible_trajectory;
				
			}
			
		}
		
	}
	
	// select behavior for lowest cost trajectory and update trajectory accordingly
	this->state.SetBehavior(best_behavior, NO_STEP_INCREASE);
	this->trajectory = best_trajectory;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_DRIVER_PLANBEHAVIOR) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  finished_steps: " << finished_steps << endl;
		cout << "  next_possible_behaviors: " << endl << this->state.CreateBehaviorVectorString(next_possible_behaviors);
		cout << "  best_behavior: " << endl << this->state.CreateBehaviorString(best_behavior);
		cout << "  best_trajectory: " << endl << best_trajectory.CreateString();
		cout << "--- DRIVER: PlanBehavior - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// get map object
Map* Driver::Get_map() {
	
	return &this->map;
	
}

// get ego object
Vehicle* Driver::Get_ego() {
	
	return &this->ego;
	
}

// set vehicles object vector
void Driver::Set_vehicles(vector<Vehicle> vehicles) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_DRIVER_SETVEHICLES) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "Driver: Set_vehicles - Start" << endl;
		cout << "  vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(vehicles);
		
	}
	
	this->vehicles = vehicles;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_DRIVER_SETVEHICLES) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this->vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(this->vehicles);
		cout << "--- DRIVER: Set_vehicles - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// get list of vehicle objects
vector<Vehicle>* Driver::Get_vehicles() {
	
	return &this->vehicles;
	
}

// get state object
State* Driver::Get_state() {
	
	return &this->state;
	
}

// get x values of path
vector<double> Driver::Get_next_x() {
	
	return this->next_x_vals;
	
}

// get y values of path
vector<double> Driver::Get_next_y() {
	
	return this->next_y_vals;
	
}

// get previous_path object
Path* Driver::Get_previous_path() {
	
	return &this->previous_path;
	
}