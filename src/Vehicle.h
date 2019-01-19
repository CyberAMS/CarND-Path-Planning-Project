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

// define constants
const unsigned int EGO_CAR_ID = 0;
const double EGO_CAR_SV_INIT = 0.0;
const double EGO_CAR_SA_INIT = 0.0;
const double EGO_CAR_SJ_INIT = 0.0;
const double EGO_CAR_DV_INIT = 0.0;
const double EGO_CAR_DA_INIT = 0.0;
const double EGO_CAR_DJ_INIT = 0.0;

class Vehicle {

public:
	
	// constructor
	Vehicle();
	Vehicle(Map map, unsigned int id, double x, double y, double vx, double vy, double s, double d);
	Vehicle(Map map, double x, double y, double s, double d, double theta, double v);
	
	// destructor
	~Vehicle() {}
	
	// set state of car
	void Update(Map map, unsigned int id, double x, double y, double vx, double vy, double s, double d);
	void Update(Map map, double x, double y, double s, double d, double theta, double v);
	void Update(Map map);
	
	// predict future trajectory
	void PredictTrajectory(Map map, const double &s_start, const double &sv_start, const double &d_start, const double &dv_start);
	
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
	
	// set trajectory
	void SetTrajectory(Trajectory trajectory);
	
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

};

#endif /* VEHICLE_H_ */