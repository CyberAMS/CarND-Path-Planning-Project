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
#include <cmath>
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
using std::min;

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
	double cost;
	double minimal_cost = std::numeric_limits<double>::max();
	behavior_state best_behavior = this->Get_state().Get_behavior();
	Trajectory best_trajectory = this->Get_ego().Get_trajectory();
	
	finished_steps = this->Get_ego_ptr()->Get_trajectory_ptr()->Init(this->Get_map(), this->Get_ego().Get_s(), EGO_CAR_SV_INIT, this->Get_ego().Get_d(), EGO_CAR_DV_INIT, EGO_CAR_DA_INIT, EGO_CAR_DJ_INIT, this->Get_ego().Get_theta(), this->Get_previous_path());
	this->Get_state_ptr()->Init(finished_steps);
	
	// get next possible states
	next_possible_behaviors = this->Get_state().GetNextPossibleBehaviors(this->Get_ego().Get_lane());
	
	// generate trajectories for all states
	for (count = 0; count < next_possible_behaviors.size(); count++) {
		
		// generate trajectory for current state
		next_possible_trajectory = this->Get_state().GenerateTrajectoryFromBehavior(this->Get_map(), this->Get_ego(), next_possible_behaviors[count]);
		
		// check whether trajectory is valid
		if (next_possible_trajectory.Valid(this->Get_map())) {
			
			// determine cost of trajectory
			cost = next_possible_trajectory.Cost(this->Get_ego(), this->Get_vehicles());
			
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
	this->Get_state_ptr()->SetBehavior(best_behavior, NO_STEP_INCREASE);
	this->Get_ego_ptr()->SetTrajectory(best_trajectory);
	
	// save next xy values
	this->next_x_vals = this->Get_ego().Get_trajectory().Get_x();
	this->next_y_vals = this->Get_ego().Get_trajectory().Get_y();
	
	// display message if required
	if (bDISPLAY && bDISPLAY_DRIVER_PLANBEHAVIOR) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  finished_steps: " << finished_steps << endl;
		cout << "  next_possible_behaviors: " << endl << this->Get_state().CreateBehaviorVectorString(next_possible_behaviors);
		cout << "  best_behavior: " << endl << this->Get_state().CreateBehaviorString(best_behavior);
		cout << "  best_trajectory: " << endl << best_trajectory.CreateString();
		cout << "  next_x_vals, next_y_vals: " << endl << CreateDoubleVectorsString((vector<vector<double>>){next_x_vals, next_y_vals});
		cout << "--- DRIVER: PlanBehavior - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
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
		cout << "  this->vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(this->Get_vehicles());
		cout << "--- DRIVER: Set_vehicles - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// get map object
Map Driver::Get_map() {
	
	return this->map;
	
}
Map* Driver::Get_map_ptr() {
	
	return &this->map;
	
}

// get ego object
Vehicle Driver::Get_ego() {
	
	return this->ego;
	
}
Vehicle* Driver::Get_ego_ptr() {
	
	return &this->ego;
	
}

// get list of vehicle objects
vector<Vehicle> Driver::Get_vehicles() {
	
	return this->vehicles;
	
}
vector<Vehicle>* Driver::Get_vehicles_ptr() {
	
	return &this->vehicles;
	
}

// get state object
State Driver::Get_state() {
	
	return this->state;
	
}
State* Driver::Get_state_ptr() {
	
	return &this->state;
	
}

// get x values of path
vector<double> Driver::Get_next_x() {
	
	return this->next_x_vals;
	
}
vector<double>* Driver::Get_next_x_ptr() {
	
	return &this->next_x_vals;
	
}

// get y values of path
vector<double> Driver::Get_next_y() {
	
	return this->next_y_vals;
	
}
vector<double>* Driver::Get_next_y_ptr() {
	
	return &this->next_y_vals;
	
}

// get previous_path object
Path Driver::Get_previous_path() {
	
	return this->previous_path;
	
}Path* Driver::Get_previous_path_ptr() {
	
	return &this->previous_path;
	
}