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
#include <vector>
#include "State.h"

using std::vector;

// constructor
State::State() {}
State::State(behavior_state behavior, unsigned int current_lane, unsigned int target_lane, unsigned int init_time) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_CONSTRUCTOR) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "STATE: Constructor - Start" << endl;
		cout << "  behavior_state: " << endl << CreateBehaviorString(behavior_state);
		cout << "  current_lane: " << current_lane << endl;
		cout << "  target_lane: " << target_lane << endl;
		cout << "  init_time: " << init_time << endl;
		
	}
	
	this->behavior = behavior;
	this->current_lane = current_lane;
	this->target_lane = target_lane;
	this->current_time = init_time;
	this->no_change_before_time = init_time;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_CONSTRUCTOR) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- STATE: Constructor - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// set state
void State::SetBehavior(behavior_state behavior, unsigned int current_lane, unsigned int target_lane, unsigned int no_change_before_time) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_STATE_SETBEHAVIOR) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "STATE: SetBehavior - Start" << endl;
		cout << "  behavior_state: " << endl << CreateBehaviorString(behavior_state);
		cout << "  current_lane: " << current_lane << endl;
		cout << "  target_lane: " << target_lane << endl;
		cout << "  no_change_before_time: " << no_change_before_time << endl;
		
	}
	
	this->behavior = behavior;
	this->current_lane = current_lane;
	this->target_lane = target_lane;
	++this->current_time;
	this->no_change_before_time = no_change_before_time;
	
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
		if (this->behavior.name == TRANSITIONS[count_t].name) {
			
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
		cout << "  next_behaviors: " << endl << CreateBehaviorVectorString(next_behaviors);
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
	text += DISPLAY_PREFIX + "behavior =\n" + CreateBehaviorString(this->behavior);
	text += DISPLAY_PREFIX;
	text += "current_lane = " + to_string(this->current_lane) + " ";
	text += "target_lane = " + to_string(this->target_lane) + " ";
	text += "current_time = " + to_string(this->current_time) + " ";
	text += "no_change_before_time = " + to_string(this->no_change_before_time) + "\n";
	
	// return output
	return text;
	
}