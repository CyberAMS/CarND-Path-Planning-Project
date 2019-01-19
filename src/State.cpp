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
#include <cmath>
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
using std::min;

// initialize state
void State::Init(unsigned long add_step) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_INIT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "STATE: Init - Start" << endl;
		
	}
	
	if (!this->is_initialized) {
		
		// set initial state
		this->SetBehavior(INITIAL_STATE, add_step);
		
		// initialization done
		this->is_initialized = true;
		
	} else {
		
		// set state (update executed steps and increase step by one)
		this->SetBehavior(this->Get_behavior(), add_step);
		
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
void State::SetBehavior(behavior_state new_behavior, unsigned long add_step) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_SETBEHAVIOR) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "STATE: SetBehavior - Start" << endl;
		cout << "  new_behavior: " << endl << this->CreateBehaviorString(new_behavior);
		cout << "  add_step: " << add_step << endl;
		
	}
	
	// define variables
	unsigned long current_step;
	
	// determine next step
	current_step = this->Get_current_step() + add_step;
	
	// check whether transitions must be locked to ensure complete state change
	if (((this->Get_behavior().lateral_state == PREPARE_LANE_CHANGE_LEFT) && (new_behavior.lateral_state == CHANGE_LANE_LEFT)) || ((this->Get_behavior().lateral_state == PREPARE_LANE_CHANGE_RIGHT) && (new_behavior.lateral_state == CHANGE_LANE_RIGHT))) {
		
		// determine and set step when transition will be finished
		this->no_change_before_step = current_step + LANE_CHANGE_TRANSITION_TIME;
		
	}
	
	// set behavior state
	this->behavior = new_behavior;
	this->current_step = current_step;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_SETBEHAVIOR) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- STATE: SetBehavior - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// get next possible states
vector<behavior_state> State::GetNextPossibleBehaviors(Vehicle ego) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_GETNEXTPOSSIBLEBEHAVIORS) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "STATE: GetNextPossibleBehaviors - Start" << endl;
		cout << "  ego: " << endl << ego.CreateString();
		
	}
	
	// define variables
	unsigned int current_lane = ego.Get_lane();
	bool can_move_left = false;
	bool can_move_right = false;
	unsigned int count_t = 0;
	vector<behavior_state> next_potential_behaviors;
	unsigned int count_b = 0;
	LONGITUDINALSTATE potential_longitudinal_state;
	LATERALSTATE potential_lateral_state;
	
	// initialize outputs
	vector<behavior_state> next_possible_behaviors;
	
	// check whether we still execute a transition and no behavior change is allowed
	if (this->Get_current_step() >= this->Get_no_change_before_step()) {
		
		// check possible lane changes
		if (current_lane > LANES[0]) {
			
			can_move_left = true;
			
		}
		if (current_lane < LANES[LANES.size() - 1]) {
			
			can_move_right = true;
			
		}
		
		// check all possible transitions
		for (count_t = 0; count_t < TRANSITIONS.size(); count_t++) {
			
			// check whether current transition defines next states for current behavior state
			if (this->Get_behavior().lateral_state == TRANSITIONS[count_t].name) {
				
				// get list of possible next behaviors
				next_potential_behaviors = TRANSITIONS[count_t].next;
				
				// check all possible behaviors for feasible lanes changes
				for (count_b = 0; count_b < next_potential_behaviors.size(); count_b++) {
					
					// get potential lateral state
					potential_longitudinal_state = next_potential_behaviors[count_b].longitudinal_state;
					potential_lateral_state = next_potential_behaviors[count_b].lateral_state;
					
					// avoid behavior that makes the vehicle drive backwards
					if (!((ego.Get_v() <= 0) && (potential_longitudinal_state == DECELERATE))) {
					
						// only include lateral changes if there are valid lanes for it or no lane change necessary
						if ((((potential_lateral_state == PREPARE_LANE_CHANGE_LEFT) || (potential_lateral_state == CHANGE_LANE_LEFT)) && can_move_left) || 
								(((potential_lateral_state == PREPARE_LANE_CHANGE_RIGHT) || (potential_lateral_state == CHANGE_LANE_RIGHT)) && can_move_right) ||
								(potential_lateral_state == KEEP_LANE)) {
							
							next_possible_behaviors.push_back(next_potential_behaviors[count_b]);
							
						}
						
					}
					
				}
				
			}
			
		}
		
	} else {
		
		// keep previous behavior
		next_possible_behaviors = (vector<behavior_state>){this->Get_behavior()};
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_GETNEXTPOSSIBLEBEHAVIORS) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  can_move_left: " << can_move_left << endl;
		cout << "  can_move_right: " << can_move_right << endl;
		cout << "  next_possible_behaviors: " << endl << this->CreateBehaviorVectorString(next_possible_behaviors);
		cout << "--- STATE: GetNextPossibleBehaviors - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	return next_possible_behaviors;
	
	}
	
}

// generate new trajectory from behavior
Trajectory State::GenerateTrajectoryFromBehavior(Map map, Vehicle ego, behavior_state behavior) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_GENERATETRAJECTORYFROMBEHAVIOR) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "STATE: GenerateTrajectoryFromBehavior - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		cout << "  ego: " << endl << ego.CreateString();
		cout << "  behavior: " << endl << CreateBehaviorString(behavior);
		
	}
	
	// define variables
	double sv_continue = ego.Get_trajectory().Get_sv()[ego.Get_trajectory().Get_sv().size() - 1];
	double s_target = 0.0;
	double sv_target = 0.0;
	double sa_target = 0.0;
	double d_target = 0.0;
	double dv_target = 0.0;
	double da_target = 0.0;
	unsigned int current_lane = ego.Get_lane();
	unsigned int intended_lane = 0;
	
	// initialize outputs
	Trajectory new_trajectory;
	
	// determine target values based on longitudinal behavior
	switch (behavior.longitudinal_state) {
		
		case ACCELERATE:
			
			// check for starting from zero speed
			if (sv_continue < TARGET_SPEED_FROM_ZERO) {
				
				sv_target = TARGET_SPEED_FROM_ZERO;
				
			} else {
				
				// set target speed
				sv_target = sv_continue + (NORMAL_ACCELERATION_S * STEP_TIME_INTERVAL);
				
			}
			break; // switch
			
		case KEEP_SPEED:
			
			// set target speed
			sv_target = sv_continue;
			break; // switch
			
		case DECELERATE:
			
			// set target speed
			sv_target = sv_continue + (NORMAL_DECELERATION_S * STEP_TIME_INTERVAL);
			break; // switch
			
	}
	
	// ensure speed limit is kept
	sv_target = min(sv_target, MAX_SPEED);
	
	// determine position after next time interval
	//s_target = (ego.Get_trajectory().Get_s()[ego.Get_trajectory().Get_s().size() - 1] + (Average((vector<double>){sv_target, sv_continue}) * STEP_TIME_INTERVAL));
	s_target = (ego.Get_trajectory().Get_s()[ego.Get_trajectory().Get_s().size() - 1] + (sv_target * STEP_TIME_INTERVAL));
	
	// determine target values based on lateral behavior
	switch (behavior.lateral_state) {
	
		case KEEP_LANE:
			
			intended_lane = current_lane;
			d_target = ego.GetLaneD(current_lane);
			break; // switch
			
		case PREPARE_LANE_CHANGE_LEFT:
			
			intended_lane = current_lane - 1;
			d_target = ego.GetLaneD(current_lane);
			break; // switch
			
		case PREPARE_LANE_CHANGE_RIGHT:
			
			intended_lane = current_lane + 1;
			d_target = ego.GetLaneD(current_lane);
			break; // switch
			
		case CHANGE_LANE_LEFT:
			
			intended_lane = current_lane - 1;
			d_target = ego.GetLaneD(intended_lane);
			break; // switch
			
		case CHANGE_LANE_RIGHT:
			
			intended_lane = current_lane + 1;
			d_target = ego.GetLaneD(intended_lane);
			break; // switch
			
	}
	
	// save final lane intended by behavior
	new_trajectory.Set_intended_lane(intended_lane);
	
	// generate trajectory steps
	new_trajectory.Generate(map, ego.Get_trajectory(), s_target, sv_target, sa_target, d_target, dv_target, da_target);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_GENERATETRAJECTORYFROMBEHAVIOR) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  current_lane: " << current_lane << endl;
		cout << "  s_target: " << s_target << endl;
		cout << "  sv_target: " << sv_target << endl;
		cout << "  sa_target: " << sa_target << endl;
		cout << "  d_target: " << d_target << endl;
		cout << "  dv_target: " << dv_target << endl;
		cout << "  da_target: " << da_target << endl;
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  new_trajectory: " << endl << new_trajectory.CreateString();
			
		}
		cout << "--- STATE: GenerateTrajectoryFromBehavior - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return new_trajectory;
	
}

// get behavior state
behavior_state State::Get_behavior() {
	
	return this->behavior;
	
}
behavior_state* State::Get_behavior_ptr() {
	
	return &this->behavior;
	
}

// get current step
unsigned long State::Get_current_step() {
	
	return this->current_step;
	
}
unsigned long* State::Get_current_step_ptr() {
	
	return &this->current_step;
	
}

// get no_change_before_step value
unsigned long State::Get_no_change_before_step() {
	
	return this->no_change_before_step;
	
}
unsigned long* State::Get_no_change_before_step_ptr() {
	
	return &this->no_change_before_step;
	
}

// get initialization status
bool State::Get_is_initialized() {
	
	return this->is_initialized;
	
}
bool* State::Get_is_initialized_ptr() {
	
	return &this->is_initialized;
	
}

// display State object as string
string State::CreateString() {
	
	//define variables
	string text = "";
	
	// add information about car to string
	text += DISPLAY_PREFIX + "behavior =\n" + this->CreateBehaviorString(this->Get_behavior());
	text += DISPLAY_PREFIX;
	text += "current_step = " + to_string(this->Get_current_step()) + " ";
	text += "no_change_before_step = " + to_string(this->Get_no_change_before_step()) + " ";
	text += "is_initialized = " + to_string(this->Get_is_initialized()) + "\n";
	
	// return output
	return text;
	
}

// display behavior_state structure as string
string State::CreateBehaviorString(const behavior_state &behavior) {
	
	//define variables
	string text = "";
	
	// get state values
	LONGITUDINALSTATE longitudinal_state = behavior.longitudinal_state;
	string longitudinal_state_text = "";
	LATERALSTATE lateral_state = behavior.lateral_state;
	string lateral_state_text = "";
	
	switch (longitudinal_state) {
		
		case ACCELERATE:
			
			longitudinal_state_text = "ACCELERATE";
			break; // switch
			
		case KEEP_SPEED:
			
			longitudinal_state_text = "KEEP_SPEED";
			break; // switch
			
		case DECELERATE:
			
			longitudinal_state_text = "DECELERATE";
			break; // switch
			
	}
	
	switch (lateral_state) {
	
		case KEEP_LANE:
			
			lateral_state_text = "KEEP_LANE";
			break; // switch
			
		case PREPARE_LANE_CHANGE_LEFT:
			
			lateral_state_text = "PREPARE_LANE_CHANGE_LEFT";
			break; // switch
			
		case PREPARE_LANE_CHANGE_RIGHT:
			
			lateral_state_text = "PREPARE_LANE_CHANGE_RIGHT";
			break; // switch
			
		case CHANGE_LANE_LEFT:
			
			lateral_state_text = "CHANGE_LANE_LEFT";
			break; // switch
			
		case CHANGE_LANE_RIGHT:
			
			lateral_state_text = "CHANGE_LANE_RIGHT";
			break; // switch
			
	}
	
	// add information about cars to string
	text += DISPLAY_PREFIX;
	text += "longitudinal_state = " + longitudinal_state_text + " ";
	text += "lateral_state = " + lateral_state_text + "\n";
	
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