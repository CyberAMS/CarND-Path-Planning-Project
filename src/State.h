/*
 * State.h
 *
 * State class
 *
 * Created on 01/13/2019
 * Author: Andre Strobel
 *
 */

#ifndef STATE_H_
#define STATE_H_

#include <iostream>
#include <vector>
#include "collision_detector.h"

using std::vector;

// define variables
enum LONGITUDINALSTATE {ACCELERATE, KEEP_SPEED, DECELERATE};
enum LATERALSTATE {KEEP_LANE, PREPARE_LANE_CHANGE_LEFT, PREPARE_LANE_CHANGE_RIGHT, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT};
struct behavior_state {
	
	LONGITUDINALSTATE longitudinal_state;
	LATERALSTATE lateral_state;
	
};
struct transition {
	
	LATERALSTATE name
	vector<behavior_state> next
	
};
vector<transition> TRANSITIONS
	{{.name = KEEP_LANE,
	  .next = {{.longitudinal_state = ACCELERATE, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = DECELERATE, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = PREPARE_LANE_CHANGE_LEFT},
	           {.longitudinal_state = DECELERATE, .lateral_state = PREPARE_LANE_CHANGE_LEFT},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = PREPARE_LANE_CHANGE_RIGHT},
	           {.longitudinal_state = DECELERATE, .lateral_state = PREPARE_LANE_CHANGE_RIGHT}}},
	 {.name = PREPARE_LANE_CHANGE_LEFT,
	  .next = {{.longitudinal_state = KEEP_SPEED, .lateral_state = CHANGE_LANE_LEFT},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = PREPARE_LANE_CHANGE_LEFT},
	           {.longitudinal_state = DECELERATE, .lateral_state = PREPARE_LANE_CHANGE_LEFT}}},
	 {.name = PREPARE_LANE_CHANGE_RIGHT,
	  .next = {{.longitudinal_state = KEEP_SPEED, .lateral_state = CHANGE_LANE_RIGHT},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = PREPARE_LANE_CHANGE_RIGHT},
	           {.longitudinal_state = DECELERATE, .lateral_state = PREPARE_LANE_CHANGE_RIGHT}}},
	 {.name = CHANGE_LANE_LEFT,
	  .next = {{.longitudinal_state = KEEP_SPEED, .lateral_state = KEEP_LANE}}},
	 {.name = CHANGE_LANE_RIGHT,
	  .next = {{.longitudinal_state = KEEP_SPEED, .lateral_state = KEEP_LANE}}}};

class State {

public:
	
	// constructor
	State();
	State(behavior_state behavior, unsigned int current_lane, unsigned int target_lane, unsigned int init_time);
	
	// destructor
	~State() {}
	
	// set state
	void SetBehavior(behavior_state behavior, unsigned int no_change_before_time);
	
	// get next possible states
	vector<behavior_state> GetNextPossibleBehaviors();
	
	// get behavior state
	behavior_state Get_behavior();
	
	// display State object as string
	string CreateString();

private:
	
	// define state variable
	behavior_state behavior;
	
	// define lane variables
	unsigned int current_lane;
	unsigned int target_lane;
	
	// define time variables
	unsigned int current_time;
	unsigned int no_change_before_time;

};

#endif /* STATE_H_ */