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
#include "helper_functions.h"

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
	
	// set trajectory
	void set(const vector<double> &s_values, const vector<double> &d_values, const vector<double> &v_values, const double &max_acceleration_s, const double &sample_time);
	
	// get trajectory s values
	vector<double> get_s();
	
	// get trajectory d values
	vector<double> get_d();
	
	// get trajectory v values
	vector<double> get_v();
	
	// get distance from center lane for lane
	vector<double> calculate_d_from_lane(const vector<unsigned int> &lane_values, const double &lane_width);
	
	// transform from Frenet s,d coordinates to Cartesian x,y
	vector<vector<double>> get_xy(const vector<double> &s_values, const vector<double> &d_values, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

private:
	
	// trajectory values
	vector<double> s_values;
	vector<double> d_values;
	vector<double> v_values;

};

#endif /* TRAJECTORY_H_ */