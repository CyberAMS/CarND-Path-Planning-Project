/*
 * Trajectory.cpp
 *
 * Trajectory class
 *
 * Created on 01/04/2019
 * Author: Andre Strobel
 *
 */

#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include "Trajectory.h"

using std::vector;
using std::string;

// set trajectory
void Trajectory::set(vector<double> s_values, vector<double> d_values, vector<double> v_values) {
	
	Trajectory::s_values = s_values;
	Trajectory::d_values = d_values;
	Trajectory::v_values = v_values;
	
}

// get trajectory s values
vector<double> Trajectory::get_s() {
	
	return Trajectory::s_values;
	
}

// get trajectory d values
vector<double> Trajectory::get_d() {
	
	return Trajectory::d_values;
	
}

// get trajectory v values
vector<double> Trajectory::get_v() {
	
	return Trajectory::v_values;
	
}

// get distance from center lane for lane
vector<double> Trajectory::get_d_from_lane(const vector<double> &lane_values, const double &lane_width) {
	
	// define variables
	unsigned int count = 0;
	double d = 0;
	
	// initialize outputs
	vector<double> d_values;
	
	for (count = 0; count < lane_values.size(); count++) {
		
		d = (lane_width / 2) + (lane_width * (lane_values[count] - 1));
		d_values.push_back(d);
		
	}
	
	return d_values;
	
}

// transform from Frenet s,d coordinates to Cartesian x,y
vector<vector<double>> Trajectory::get_xy(const vector<double> &s_values, const vector<double> &d_values, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	// define variables
	unsigned int count = 0;
	double s = 0;
	double d = 0;
	
	// initialize outputs
	vector<double> x_values;
	vector<double> y_values;
	
	// process all points
	for (count = 0; count < s_values.size(); count++) {
		
		s = s_values[count];
		d = d_values[count];
		
		int prev_wp = -1;
		
		while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
			
			prev_wp++;
			
		}
		
		int wp2 = (prev_wp+1)%maps_x.size();
		
		double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
		// the x,y,s along the segment
		double seg_s = (s-maps_s[prev_wp]);
		
		double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
		double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
		
		double perp_heading = heading-M_PI/2;
		
		double x = seg_x + d*cos(perp_heading);
		double y = seg_y + d*sin(perp_heading);
		
		// save current point
		x_values.push_back(x);
		y_values.push_back(y);
		
	}
	
	return {x_values, y_values};
	
}