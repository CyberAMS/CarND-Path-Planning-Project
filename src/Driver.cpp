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
	unsigned long finished_steps = 0;
	vector<behavior_state> next_possible_behaviors;
	unsigned int count = 0;
	Trajectory next_possible_trajectory;
	vector<Trajectory> next_possible_trajectories;
	
	// initialize all objects for next step
	finished_steps = this->trajectory.Init(this->ego, this->previous_path);
	this->state.Init(this->ego, this->trajectory, finished_steps);
	
	// get next possible states
	next_possible_behaviors = state.GetNextPossibleBehaviors();
	
	// generate trajectories for all states
	for (count = 0; count < next_possible_behaviors.size(); count++) {
		
		next_possible_trajectory.Generate();
		
		if (next_possible_trajectory.Valid()) {
			
			next_possible_trajectories.push_back(next_possible_trajectory);
			
		}
		
	}
	
	// evaluate cost for all trajectories
	
	// select lowest cost trajectory
	state.SetBehavior(best_behavior, current_lane, target_lane, NO_STEP_INCREASE);
	
	// generate path
	
	
	// display message if required
	if (bDISPLAY && bDISPLAY_DRIVER_PLANBEHAVIOR) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  finished_steps: " << finished_steps << endl;
		cout << "--- DRIVER: PlanBehavior - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// get map object
Map Driver::Get_map() {
	
	return this->map;
	
}

// get ego object
Vehicle Driver::Get_ego() {
	
	return this->ego;
	
}

// set vehicles object vector
void Driver::Set_vehicles(vector<Vehicle> vehicles) {
	
	this->vehicles = vehicles;
	
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
Path Driver::Get_previous_path() {
	
	this->previous_path;
	
}