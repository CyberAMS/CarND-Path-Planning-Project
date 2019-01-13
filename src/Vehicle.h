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
#include <string>
#include <vector>

using std::vector;
using std::string;

// define constants
const unsigned int EGO_CAR_ID = 0;

class Vehicle {

public:
	
	// constructor
	Vehicle();
	Vehicle(unsigned int id, double x, double y, double vx, double vy, double s, double d);
	Vehicle(double x, double y, double s, double d, double theta, double v);
	
	// destructor
	~Vehicle() {}
	
	// set state of car
	void Update(unsigned int id, double x, double y, double vx, double vy, double s, double d);
	void Update(double x, double y, double s, double d, double theta, double v);
	void Update();
	
	// get vehicles ahead of own vehicle
	vector<Vehicle> Ahead(const vector<Vehicle> &vehicles, unsigned int lane);
	
	// get vehicles behind own vehicle
	vector<Vehicle> Behind(const vector<Vehicle> &vehicles, unsigned int lane);
	
	// get x value
	double Get_x();
	
	// get y value
	double Get_y();
	
	// get vx value
	double Get_vx();
	
	// get vy value
	double Get_vy();
	
	// get s value
	double Get_s();
	
	// get d value
	double Get_d();
	
	// get yaw angle
	double Get_theta();
	
	// get speed
	double Get_v();
	
	// get lane value
	unsigned int Get_lane();
	
	// get is_inside_lane value
	bool Get_is_inside_lane();
	
	// display Vehicle object as string
	string CreateString();

private:
	
	// get d values for lanes
	vector<double> GetLaneD(const vector<unsigned int> &lanes);
	
	// determine lane
	unsigned int DetermineLane();
	
	// check whether vehicle is inside the determined lane
	bool CheckInsideLane();
	
	// state values
	double id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
	double theta;
	double v;
	
	// determined values
	unsigned int lane;
	bool is_inside_lane;

};

#endif /* VEHICLE_H_ */