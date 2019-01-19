/*
 * Vehicle.h
 *
 * Vehicle class
 *
 * Created on 01/13/2019
 * Author: Andre Strobel
 *
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "Trajectory.h"

using std::vector;
using std::string;

// vehicle parameters
const unsigned int EGO_CAR_ID = 0;
const double EGO_CAR_SV_INIT = 0.0;
const double EGO_CAR_SA_INIT = 0.0;
const double EGO_CAR_SJ_INIT = 0.0;
const double EGO_CAR_DV_INIT = 0.0;
const double EGO_CAR_DA_INIT = 0.0;
const double EGO_CAR_DJ_INIT = 0.0;
const double STANDARD_VEHICLE_WIDTH = 2.0;
const double STANDARD_VEHICLE_LENGTH = 4.0;
const double SAFETY_BOX_DISTANCE = 0.5; // must have 0.5 m distance to all vehicles around own vehicle

// cost parameters
const double DESIRED_LONGITUDINAL_TIME_DISTANCE = 1.0; // keep a distance of 1 s
const double NO_HARMFUL_COLLISION_STEPS = DESIRED_LONGITUDINAL_TIME_DISTANCE / SAMPLE_TIME;
const double COST_STEPS_TO_COLLISION_SHAPE_FACTOR = 10.0;
const double AHEAD_SPACE_FACTOR = 2.0;
const double BEHIND_SPACE_FACTOR = 4.0;
const double VEHICLE_AHEAD_WITHIN_DISTANCE = 50.0;
const double COST_SPEED_IN_INTENDED_LANE_SHAPE_FACTOR = 10.0;
const double MAX_TRAVEL_DISTANCE = MAX_SPEED * STEP_TIME_INTERVAL;
const double COST_TRAVEL_DISTANCE_SHAPE_FACTOR = 10.0;

// cost weights
const double ZERO_COST = 0.0;
const double MAX_NORMALIZED_COST = 1.0;
const double COST_COLLISON_WEIGHT = 10.0;
const double COST_SPACEININTENDEDLANE_WEIGHT = 5.0;
const double COST_SPEEDININTENDEDLANE_WEIGHT = 1.0;
const double COST_TRAVELDISTANCE_WEIGHT = 1.0;

class Vehicle {

public:
	
	// constructor
	Vehicle();
	Vehicle(Map map, unsigned int id, double x, double y, double vx, double vy, double s, double d);
	Vehicle(double x, double y, double s, double d, double theta, double v);
	
	// destructor
	~Vehicle() {}
	
	// set state of car
	void Update(Map map, unsigned int id, double x, double y, double vx, double vy, double s, double d);
	void Update(double x, double y, double s, double d, double theta, double v);
	void Update();
	
	// predict future trajectory
	void PredictTrajectory(Map map, const double &s_start, const double &sv_start, const double &d_start, const double &dv_start, const double &theta_start);
	
	// get d values for lanes
	double GetLaneD(const unsigned int &lane);
	vector<double> GetLaneD(const vector<unsigned int> &lanes);
	
	// determine lane
	unsigned int DetermineLane(const double &d);
	unsigned int DetermineLane();
	
	// check whether vehicle is inside the determined lane
	bool CheckInsideLane(const double &d, const unsigned int &lane);
	bool CheckInsideLane();
	
	// get vehicles ahead of own vehicle
	vector<Vehicle> Ahead(const vector<Vehicle> &vehicles, const unsigned int &lane);
	
	// get vehicles behind own vehicle
	vector<Vehicle> Behind(const vector<Vehicle> &vehicles, const unsigned int &lane);
	
	// detect collision between trajectory of own vehicle and trajectories of other vehicles
	unsigned long DetectCollision(Trajectory trajectory, vector<Vehicle> vehicles);
	unsigned long DetectCollision(vector<Vehicle> vehicles);
	
	// determine collison cost
	double CostStepsToCollision(Trajectory trajectory, vector<Vehicle> vehicles, const double &weight);
	
	// determine whether there is enough space in the intended lane
	double CostSpaceInIntendedLane(Trajectory trajectory, vector<Vehicle> vehicles, const double &weight);
	
	// determine cost for speed in intended lane
	double CostSpeedInIntendedLane(Trajectory trajectory, vector<Vehicle> vehicles, const double &weight);
	
	// determine cost for travel distance
	double CostTravelDistance(Trajectory trajectory, const double &weight);
	
	// determine cost of trajectory for own vehicle
	double TrajectoryCost(Trajectory trajectory, vector<Vehicle> vehicles);
	
	// set trajectory
	void Set_trajectory(Trajectory trajectory);
	
	// get id number
	unsigned int Get_id();
	unsigned int* Get_id_ptr();
	
	// get x value
	double Get_x();
	double* Get_x_ptr();
	
	// get y value
	double Get_y();
	double* Get_y_ptr();
	
	// get vx value
	double Get_vx();
	double* Get_vx_ptr();
	
	// get vy value
	double Get_vy();
	double* Get_vy_ptr();
	
	// get s value
	double Get_s();
	double* Get_s_ptr();
	
	// get d value
	double Get_d();
	double* Get_d_ptr();
	
	// get yaw angle
	double Get_theta();
	double* Get_theta_ptr();
	
	// get speed
	double Get_v();
	double* Get_v_ptr();
	
	// get lane value
	unsigned int Get_lane();
	unsigned int* Get_lane_ptr();
	
	// get is_inside_lane value
	bool Get_is_inside_lane();
	bool* Get_is_inside_lane_ptr();
	
	// get trajectory
	Trajectory Get_trajectory();
	Trajectory* Get_trajectory_ptr();
	
	// get width
	double Get_width();
	double* Get_width_ptr();
	
	// get length
	double Get_length();
	double* Get_length_ptr();
	
	// display Vehicle object as string
	string CreateString();
	
	// display vector of Vehicle objects as string
	string CreateVehiclesVectorString(vector<Vehicle> vehicles_vector);

private:
	
	// state values
	unsigned int id = 0;
	double x = 0;
	double y = 0;
	double vx = 0;
	double vy = 0;
	double s = 0;
	double d = 0;
	double theta = 0;
	double v = 0;
	
	// determined values
	unsigned int lane = 0;
	bool is_inside_lane = false;
	
	// own trajectory
	Trajectory trajectory;
	
	// vehicle dimensions
	double width = STANDARD_VEHICLE_WIDTH;
	double length = STANDARD_VEHICLE_LENGTH;

};

#endif /* VEHICLE_H_ */