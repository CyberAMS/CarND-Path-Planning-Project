/*
 * State.cpp
 *
 * State class
 *
 * Created on 01/13/2019
 * Author: Andre Strobel
 *
 */

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "State.h"
#include "Map.h"
#include "Vehicle.h"
#include "Trajectory.h"

using std::vector;
using std::string;
using std::to_string;
using std::cout;
using std::endl;
using std::ostringstream;

// initialize state
void State::Init(Vehicle ego, Trajectory trajectory, unsigned long add_step) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_INIT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "STATE: Init - Start" << endl;
		cout << "  ego: " << endl << ego.CreateString();
		cout << "  trajectory: " << endl << trajectory.CreateString();
		
	}
	
	// define variables
	unsigned int current_lane = 0;
	unsigned int target_lane = 0;
	
	if (!this->is_initialized) {
		
		// determine current and target lane
		current_lane = ego.DetermineLane(trajectory.Get_d()[0]);
		target_lane = ego.DetermineLane(trajectory.Get_d()[trajectory.Get_d().size() - 1]);
		
		// set initial state
		this->SetBehavior(INITIAL_STATE, current_lane, target_lane, add_step, INITIAL_STEP);
		
		// initialization done
		this->is_initialized = true;
		
	} else {
		
		// set state (update executed steps and increase step by one)
		this->SetBehavior(this->behavior, this->current_lane, this->target_lane, add_step, this->current_step);
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_INIT) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- STATE: Init - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// set state
void State::SetBehavior(behavior_state behavior, unsigned int current_lane, unsigned int target_lane, unsigned long add_step, unsigned long no_change_before_step) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_SETBEHAVIOR) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "STATE: SetBehavior - Start" << endl;
		cout << "  behavior_state: " << endl << this->CreateBehaviorString(behavior);
		cout << "  current_lane: " << current_lane << endl;
		cout << "  target_lane: " << target_lane << endl;
		cout << "  add_step: " << add_step << endl;
		cout << "  no_change_before_step: " << no_change_before_step << endl;
		
	}
	
	this->behavior = behavior;
	this->current_lane = current_lane;
	this->target_lane = target_lane;
	this->current_step += (add_step + 1); // TODO: check whether this really needs to be increased by 1 or wait for previous_path to increment this
	this->no_change_before_step = no_change_before_step;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_SETBEHAVIOR) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- STATE: SetBehavior - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// get next possible states
vector<behavior_state> State::GetNextPossibleBehaviors() {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_GETNEXTPOSSIBLEBEHAVIORS) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "STATE: GetNextPossibleBehaviors - Start" << endl;
		
	}
	
	// define variables
	bool can_move_left = false;
	bool can_move_right = false;
	unsigned int count_t = 0;
	vector<behavior_state> next_potential_behaviors;
	unsigned int count_b = 0;
	LATERALSTATE potential_lateral_state;
	
	// initialize outputs
	vector<behavior_state> next_behaviors;
	
	// check possible lane changes
	if (this->current_lane > LANES[0]) {
		
		can_move_left = true;
		
	}
	if (this->current_lane < LANES[LANES.size() - 1]) {
		
		can_move_right = true;
		
	}
	
	// check all possible transitions
	for (count_t = 0; count_t < TRANSITIONS.size(); count_t++) {
		
		// check whether current transition defines next states for current behavior state
		if (this->behavior.lateral_state == TRANSITIONS[count_t].name) {
			
			// get list of possible next behaviors
			next_potential_behaviors = TRANSITIONS[count_t].next;
			
			// check all possible behaviors for feasible lanes changes
			for (count_b = 0; count_b < next_potential_behaviors.size(); count_b++) {
				
				// get potential lateral state
				potential_lateral_state = next_potential_behaviors[count_b].lateral_state;
				
				// only include lateral changes if there are valid lanes for it
				if ((((potential_lateral_state == PREPARE_LANE_CHANGE_LEFT) || (potential_lateral_state == CHANGE_LANE_LEFT)) && can_move_left) || ((potential_lateral_state == PREPARE_LANE_CHANGE_RIGHT) || (potential_lateral_state == CHANGE_LANE_RIGHT)) && can_move_right) {
					
					next_behaviors.push_back(next_potential_behaviors[count_b]);
					
				};
				
			}
			
		}
		
	}
	
	return next_behaviors;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_SETBEHAVIOR) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  can_move_left: " << can_move_left << endl;
		cout << "  can_move_right: " << can_move_right << endl;
		cout << "  next_behaviors: " << endl << this->CreateBehaviorVectorString(next_behaviors);
		cout << "--- STATE: GetNextPossibleBehaviors - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// get behavior state
behavior_state State::Get_behavior() {
	
	return this->behavior;
	
}

// display State object as string
string State::CreateString() {
	
	//define variables
	string text = "";
	
	// add information about car to string
	text += DISPLAY_PREFIX + "behavior =\n" + this->CreateBehaviorString(this->behavior);
	text += DISPLAY_PREFIX;
	text += "current_lane = " + to_string(this->current_lane) + " ";
	text += "target_lane = " + to_string(this->target_lane) + " ";
	text += "current_step = " + to_string(this->current_step) + " ";
	text += "no_change_before_step = " + to_string(this->no_change_before_step) + " ";
	text += "is_initialized = " + to_string(this->is_initialized) + "\n";
	
	// return output
	return text;
	
}

// display behavior_state structure as string
string State::CreateBehaviorString(const behavior_state &behavior) {
	
	//define variables
	string text = "";
	
	// add information about cars to string
	text += DISPLAY_PREFIX;
	text += "longitudinal_state = " + to_string(behavior.longitudinal_state) + " ";
	text += "lateral_state = " + to_string(behavior.lateral_state) + "\n";
	
	// return output
	return text;
	
}

// display vector of Vehicle objects as string
string State::CreateBehaviorVectorString(vector<behavior_state> behaviors_vector) {
	
	//define variables
	unsigned int current_element = 0;
	string text = "";
	
	// add information about all behaviors to string
	for (current_element = 0; current_element < behaviors_vector.size(); current_element++) {
		
		text += DISPLAY_PREFIX + "Element " + to_string(current_element) + ": " + this->CreateBehaviorString(behaviors_vector[current_element]);
		
	}
	
	// return output
	return text;
	
}