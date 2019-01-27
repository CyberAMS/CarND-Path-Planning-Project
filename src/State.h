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
#include <sstream>
#include <string>
#include <vector>
#include "Map.h"
#include "Vehicle.h"
#include "Trajectory.h"

using std::vector;
using std::string;

// define types
enum LONGITUDINALSTATE {ACCELERATE, KEEP_SPEED, DECELERATE};
enum LATERALSTATE {KEEP_LANE, PREPARE_LANE_CHANGE_LEFT, PREPARE_LANE_CHANGE_RIGHT, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT};
struct behavior_state {
	
	LONGITUDINALSTATE longitudinal_state;
	LATERALSTATE lateral_state;
	
};
struct transition {
	
	LATERALSTATE name;
	vector<behavior_state> next;
	
};

// define constants
const unsigned long INITIAL_STEP = 0;
const behavior_state INITIAL_STATE {.longitudinal_state = ACCELERATE, .lateral_state = KEEP_LANE};
const vector<transition> TRANSITIONS
	{{.name = KEEP_LANE,
	  .next = {{.longitudinal_state = DECELERATE, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = ACCELERATE, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = DECELERATE, .lateral_state = PREPARE_LANE_CHANGE_LEFT},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = PREPARE_LANE_CHANGE_LEFT},
	           {.longitudinal_state = DECELERATE, .lateral_state = PREPARE_LANE_CHANGE_RIGHT},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = PREPARE_LANE_CHANGE_RIGHT}}},
	 {.name = PREPARE_LANE_CHANGE_LEFT,
	  .next = {{.longitudinal_state = KEEP_SPEED, .lateral_state = CHANGE_LANE_LEFT},
	           {.longitudinal_state = DECELERATE, .lateral_state = PREPARE_LANE_CHANGE_LEFT},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = PREPARE_LANE_CHANGE_LEFT},
	           {.longitudinal_state = DECELERATE, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = ACCELERATE, .lateral_state = KEEP_LANE}}},
	 {.name = PREPARE_LANE_CHANGE_RIGHT,
	  .next = {{.longitudinal_state = KEEP_SPEED, .lateral_state = CHANGE_LANE_RIGHT},
	           {.longitudinal_state = DECELERATE, .lateral_state = PREPARE_LANE_CHANGE_RIGHT},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = PREPARE_LANE_CHANGE_RIGHT},
	           {.longitudinal_state = DECELERATE, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = ACCELERATE, .lateral_state = KEEP_LANE}}},
	 {.name = CHANGE_LANE_LEFT,
	  .next = {{.longitudinal_state = DECELERATE, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = ACCELERATE, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = CHANGE_LANE_LEFT}}},
	 {.name = CHANGE_LANE_RIGHT,
	  .next = {{.longitudinal_state = DECELERATE, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = ACCELERATE, .lateral_state = KEEP_LANE},
	           {.longitudinal_state = KEEP_SPEED, .lateral_state = CHANGE_LANE_RIGHT}}}};
const long LANE_CHANGE_TRANSITION_TIME = 0.5 * STEP_TIME_INTERVAL / SAMPLE_TIME; // in steps
const long NO_STEP_INCREASE = 0;

class State {

public:
	
	// constructor
	State() {}
	
	// destructor
	~State() {}
	
	// initialize state
	void Init(unsigned long add_step);
	
	// set state
	void SetBehavior(behavior_state behavior, unsigned long add_step);
	
	// get next possible states
	vector<behavior_state> GetNextPossibleBehaviors(Vehicle ego);
	
	// generate new trajectory from behavior
	Trajectory GenerateTrajectoryFromBehavior(Map map, Vehicle ego, behavior_state behavior);
	
	// get behavior state
	behavior_state Get_behavior();
	behavior_state* Get_behavior_ptr();
	
	// get current step
	unsigned long Get_current_step();
	unsigned long* Get_current_step_ptr();
	
	// get no_change_before_step value
	unsigned long Get_no_change_before_step();
	unsigned long* Get_no_change_before_step_ptr();
	
	// get initialization status
	bool Get_is_initialized();
	bool* Get_is_initialized_ptr();
	
	// display State object as string
	string CreateString();
	
	// display behavior_state structure as string
	string CreateBehaviorString(const behavior_state &behavior);
	
	// display vector of Vehicle objects as string
	string CreateBehaviorVectorString(vector<behavior_state> behaviors_vector);

private:
	
	// define state variable
	behavior_state behavior = INITIAL_STATE;
	
	// define time variables
	unsigned long current_step = INITIAL_STEP;
	unsigned long no_change_before_step = INITIAL_STEP;
	
	// remember whether state has been initialized before
	bool is_initialized = false;

};

#endif /* STATE_H_ */