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
void State::Init(Vehicle ego, Trajectory trajectory, unsigned long add_step) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_INIT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "STATE: Init - Start" << endl;
		cout << "  ego: " << endl << ego.CreateString();
		cout << "  trajectory: " << endl << trajectory.CreateString();
		
	}
	
	if (!this->is_initialized) {
		
		// set initial state
		this->SetBehavior(INITIAL_STATE, add_step);
		
		// initialization done
		this->is_initialized = true;
		
	} else {
		
		// set state (update executed steps and increase step by one)
		this->SetBehavior(this->behavior, add_step);
		
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
	unsigned long no_change_before_step;
	
	// determine next step
	current_step = this->current_step + add_step; // + 1; // TODO: check whether this really needs to be increased by 1 or wait for previous_path to increment this
	
	// check whether transitions must be locked to ensure complete state change
	if (((this->behavior.lateral_state == PREPARE_LANE_CHANGE_LEFT) && (new_behavior.lateral_state == CHANGE_LANE_LEFT)) || ((this->behavior.lateral_state == PREPARE_LANE_CHANGE_RIGHT) && (new_behavior.lateral_state == CHANGE_LANE_RIGHT))) {
		
		// determine step when transition will be finished
		no_change_before_step = current_step + LANE_CHANGE_TRANSITION_TIME;
		
	} else {
		
		// allow change in current step
		no_change_before_step = current_step;
		
	}
	
	// set behavior state
	this->behavior = new_behavior;
	this->current_step = current_step;
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
vector<behavior_state> State::GetNextPossibleBehaviors(unsigned int current_lane) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_GETNEXTPOSSIBLEBEHAVIORS) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "STATE: GetNextPossibleBehaviors - Start" << endl;
		cout << "  current_lane: " << current_lane << endl;
		
	}
	
	// define variables
	bool can_move_left = false;
	bool can_move_right = false;
	unsigned int count_t = 0;
	vector<behavior_state> next_potential_behaviors;
	unsigned int count_b = 0;
	LATERALSTATE potential_lateral_state;
	
	// initialize outputs
	vector<behavior_state> next_possible_behaviors;
	
	// check whether we still execute a transition and no behavior change is allowed
	if (this->current_step < this->no_change_before_step) {
		
		// only return current behavior state
		next_possible_behaviors = (vector<behavior_state>){this->behavior};
		
	} else {
		
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
			if (this->behavior.lateral_state == TRANSITIONS[count_t].name) {
				
				// get list of possible next behaviors
				next_potential_behaviors = TRANSITIONS[count_t].next;
				
				// check all possible behaviors for feasible lanes changes
				for (count_b = 0; count_b < next_potential_behaviors.size(); count_b++) {
					
					// get potential lateral state
					potential_lateral_state = next_potential_behaviors[count_b].lateral_state;
					
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
Trajectory State::GenerateTrajectoryFromBehavior(Map map, Vehicle ego, Trajectory trajectory, behavior_state behavior) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_GENERATETRAJECTORYFROMBEHAVIOR) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "STATE: GenerateTrajectoryFromBehavior - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		cout << "  ego: " << endl << ego.CreateString();
		cout << "  trajectory: " << endl << trajectory.CreateString();
		cout << "  behavior: " << endl << CreateBehaviorString(behavior);
		
	}
	
	// define variables
	double sv_continue = trajectory.Get_sv()[trajectory.Get_sv().size() - 1];
	double s_target = 0.0;
	double sv_target = 0.0;
	double sa_target = 0.0;
	double d_target = 0.0;
	double dv_target = 0.0;
	double da_target = 0.0;
	double speed_factor = 1.0;
	unsigned int current_lane = ego.Get_lane();
	
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
				//sv_target = sv_continue * ACCELERATION_FACTOR;
				sv_target = sv_continue + (NORMAL_ACCELERATION_S * STEP_TIME_INTERVAL);
				
			}
			break; // switch
			
		case KEEP_SPEED:
			
			// set target speed
			sv_target = sv_continue;
			break; // switch
			
		case DECELERATE:
			
			// set target speed
			//sv_target = sv_continue * DECELERATION_FACTOR;
			sv_target = sv_continue + (NORMAL_DECELERATION_S * STEP_TIME_INTERVAL);
			break; // switch
			
	}
	
	// ensure speed limit is kept
	sv_target = min(sv_target, MAX_SPEED);
	
	// determine position after next time interval
	s_target = (trajectory.Get_s()[trajectory.Get_s().size() - 1] + sv_target * STEP_TIME_INTERVAL);
	
	// determine target values based on lateral behavior
	switch (behavior.lateral_state) {
	
		case KEEP_LANE:
			
			d_target = ego.GetLaneD(current_lane);
			break; // switch
			
		case PREPARE_LANE_CHANGE_LEFT:
			
			d_target = ego.GetLaneD(current_lane);
			break; // switch
			
		case PREPARE_LANE_CHANGE_RIGHT:
			
			d_target = ego.GetLaneD(current_lane);
			break; // switch
			
		case CHANGE_LANE_LEFT:
			
			d_target = ego.GetLaneD(current_lane - 1);
			break; // switch
			
		case CHANGE_LANE_RIGHT:
			
			d_target = ego.GetLaneD(current_lane + 1);
			break; // switch
			
	}
	
	new_trajectory.Generate(map, trajectory, s_target, sv_target, sa_target, d_target, dv_target, da_target);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_GENERATETRAJECTORYFROMBEHAVIOR) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  current_lane: " << current_lane << endl;
		cout << "  speed_factor: " << speed_factor << endl;
		cout << "  s_target: " << s_target << endl;
		cout << "  sv_target: " << sv_target << endl;
		cout << "  sa_target: " << sa_target << endl;
		cout << "  d_target: " << d_target << endl;
		cout << "  dv_target: " << dv_target << endl;
		cout << "  da_target: " << da_target << endl;
		cout << "  new_trajectory: " << endl << new_trajectory.CreateString();
		cout << "--- STATE: GenerateTrajectoryFromBehavior - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return new_trajectory;
	
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