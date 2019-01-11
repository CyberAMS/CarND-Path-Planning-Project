/*
 * Trajectory.h
 *
 * Trajectory class
 *
 * Created on 01/04/2019
 * Author: Andre Strobel
 *
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "helper_functions.h"
#include "Car.h"
#include "Path.h"
#include "spline.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

class Trajectory {

public:
	
	// constructor
	Trajectory() {}
	
	// destructor
	~Trajectory() {}
	
	// init trajectory
	void init(Car myCar, Path myPreviousPath, const double &target_speed, const unsigned int &previous_path_steps, const double &sample_time, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
	
	// add new trajectory segment
	void add(const unsigned int &current_lane, const double &current_speed, const unsigned int &target_lane, const double &target_speed, const double &lane_width, const double &max_acceleration_s, const double &max_acceleration_d, const double &max_waypoint_distance, const double &back_distance, const vector<double> &maps_x, const vector<double> &maps_y);
	
	// add new trajectory segment with fixed spacing
	void add_fixed(const unsigned int &target_lane, const double &target_speed, const double &lane_width, const vector<double> &fixed_s_distances);
	
	// calculate full trajectory
	void calculate(Car myCar, const double &back_distance, const double &sample_time, const double &max_acceleration_s, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
	
	// get trajectory x values
	vector<double> get_x();
	
	// get trajectory y values
	vector<double> get_y();
	
	// get segment s values
	vector<double> get_s();
	
	// get segment d values
	vector<double> get_d();
	
	// get segment v values
	vector<double> get_v();
	
	// get distance from center lane for lane
	vector<double> calculate_d_from_lane(const vector<unsigned int> &lane_values, const double &lane_width);
	
	// estimate lane number for lane based on distance from center lane
	vector<unsigned int> estimate_lanes(const vector<double> &d_values, const vector<unsigned int> &lanes, const double &lane_width);
	
	// determine closest waypoint
	vector<unsigned int> ClosestWaypoint(const double &x, const double &y, const vector<double> &maps_x, const vector<double> &maps_y);
	
	// determine next waypoint
	vector<unsigned int> NextWaypoint(const double &x, const double &y, const double &theta, const vector<double> &maps_x, const vector<double> &maps_y);
	
	// transform from Cartesian x,y coordinates to Frenet s,d coordinates
	vector<vector<double>> getFrenet(const vector<double> &x_values, const vector<double> &y_values, const double &theta, const vector<double> &maps_x, const vector<double> &maps_y);
	
	// transform from Frenet s,d coordinates to Cartesian x,y
	vector<vector<double>> get_xy(const vector<double> &s_values, const vector<double> &d_values, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

private:
	
	// trajectory values
	vector<double> x_values;
	vector<double> y_values;
	
	// new segment values
	vector<double> s_values;
	vector<double> d_values;
	vector<double> v_values;
	double connect_theta;

};

#endif /* TRAJECTORY_H_ */