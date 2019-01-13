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
#include "helper_functions.h"

//#include "Trajectory.h"

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
	
	
	
	
	
	
	//this->
	
	
	
	
	
	
	
	
	// define variables
	//vector<unsigned int> estimated_lanes;
	
	// determine vehicles around own vehicles
	//Driver::current_speed = myCar.get_v();
	//estimated_lanes = Driver::trajectory.estimate_lanes((vector<double>){myCar.get_d()}, LANES, LANE_WIDTH);
	//Driver::current_lane = estimated_lanes[0];
	// sd coordinates from xy sensors
	// predict vehicle positions in next seconds and use in the following
	// vehicle in front of own vehicle in "radar" distance (if any)
	// vehicles left and right of own vehicle in "radar" distance
	// gap to the left and right (front and back)
	// speed of lanes (slowest vehicle in front)
	
	// calculate gap distance (1.5 seconds * speed in front - also for other lanes?!?)
	
	// select follow lane behavior - should actually be a finite state logic
	Driver::behavior = BEHAVIORS[0];
	
	// determine plan based on selected behavior
	if (Driver::behavior.name == BEHAVIORS[0].name) {
		// follow lane at maximum speed
		
		// determine target speed
		//Driver::target_speed = MAX_V;
		
		// determine target lane
		//Driver::target_lane = Driver::current_lane;
		
	} else if (Driver::behavior.name == BEHAVIORS[1].name) {
		// keep lane and follow vehicle in front
		
		// determine target speed
		
		// US rules
		// min(speed in front, max speed)
		
		// European rules
		// min(spped in front and left of vehicle, max speed)
		
		// determine target lane
		// need cost function ?!?
		
	} else if (Driver::behavior.name == BEHAVIORS[2].name) {
		// prepare lane change to left
		
		// determine target speed
		
		
		// determine target lane
		
		
	} else if (Driver::behavior.name == BEHAVIORS[3].name) {
		// prepare lane change to right
		
		// determine target speed
		
		
		// determine target lane
		
		
	} else if (Driver::behavior.name == BEHAVIORS[4].name) {
		// lane change to left
		
		// determine target speed
		
		
		// determine target lane
		
		
	} else if (Driver::behavior.name == BEHAVIORS[5].name) {
		// lane change to right
		
		// determine target speed
		
		
		// determine target lane
		
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_DRIVER_PLANBEHAVIOR) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		//cout << "  Driver::behavior.name: " << Driver::behavior.name << endl;
		//cout << "  Driver::current_lane: " << Driver::current_lane << endl;
		//cout << "  Driver::current_speed: " << Driver::current_speed << endl;
		//cout << "  Driver::target_lane: " << Driver::target_lane << endl;
		//cout << "  Driver::target_speed: " << Driver::target_speed << endl;
		cout << "--- DRIVER: PlanBehavior - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

/*
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
	Driver::trajectory.init(myCar, myPreviousPath, Driver::target_speed, PREVIOUS_PATH_STEPS, SAMPLE_TIME, Driver::maps_s, Driver::maps_x, Driver::maps_y);
	
	// determine new segment based on plan
	if (USE_FIXED_DISTANCES) {
		
		Driver::trajectory.add_fixed(Driver::target_lane, Driver::target_speed, LANE_WIDTH, FIXED_S_DISTANCES);
		
	} else {
		
		Driver::trajectory.add(Driver::current_lane, Driver::current_speed, Driver::target_lane, Driver::target_speed, LANE_WIDTH, MAX_ACCELERATION_S, MAX_ACCELERATION_D, MAX_WAYPOINT_DISTANCE, BACK_DISTANCE, Driver::maps_x, Driver::maps_y);
		
	}
	
	// calculate xy trajectory from init and additional segment
	Driver::trajectory.calculate(myCar, BACK_DISTANCE, SAMPLE_TIME, MAX_ACCELERATION_S, Driver::maps_s, Driver::maps_x, Driver::maps_y);
	
	// return path based on trajectory
	Driver::next_x_vals = Driver::trajectory.get_x();
	Driver::next_y_vals = Driver::trajectory.get_y();
	
	// display message if required
	if (bDISPLAY && bDISPLAY_DRIVER_CALCULATE_TRAJECTORY) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  Driver::trajectory.get_s(), Driver::trajectory.get_d(), Driver::trajectory.get_v(): " << endl << CreateDoubleVectorsString(vector<vector<double>>{Driver::trajectory.get_s(), Driver::trajectory.get_d(), Driver::trajectory.get_v()});
		cout << "  Driver::next_x_vals, Driver::next_y_vals: " << endl << CreateDoubleVectorsString(vector<vector<double>>{Driver::next_x_vals, Driver::next_y_vals});
		cout << "--- DRIVER: calculate_trajectory - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}*/

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