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
#include "helper_functions.h"
#include "Trajectory.h"
#include "spline.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;
using std::min;
using std::max;

// set trajectory
void Trajectory::set(const vector<double> &s_values, const vector<double> &d_values, const vector<double> &v_values, const double &max_acceleration_s, const double &sample_time) {
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: set - Start" << endl;
		cout << "  s_values: " << createDoubleVectorString(s_values);
		cout << "  d_values: " << createDoubleVectorString(d_values);
		cout << "  v_values: " << createDoubleVectorString(v_values);
		cout << "  max_acceleration_s: " << max_acceleration_s << endl;
		cout << "  sample_time: " << sample_time << endl;
		
	}
	
	// define variables
	double max_speed_change = max_acceleration_s * sample_time;
	unsigned int count = 0;
	vector<double> new_s_values;
	vector<double> new_d_values;
	vector<double> new_v_values;
	double new_s_value;
	double new_d_value;
	double new_v_value;
	bool bDone = false;
	
	// create spline
	tk::spline s;
	s.set_points(s_values, d_values);
	
	// initialize new values
	new_s_values.push_back(s_values[0]);
	new_d_values.push_back(d_values[0]);
	new_v_values.push_back(v_values[0]);
	
	// calculate new values with speed steps along spline
	for (count = 1; count < s_values.size(); count ++) {
		
		// reset boolean for all interior speed steps calculated
		bDone = false;
		
		while ((new_s_values.back() < s_values[count]) && !bDone) {
			
			// next speed value is old speed value plus maximum delta to achieve next target value
			new_v_value = new_v_values.back() + max(min((v_values[count] - new_v_values.back()), max_speed_change), -max_speed_change);
			
			// next s value is old s value plus distance driven at new speed value during sample time
			new_s_value = new_s_values.back() + (new_v_value * sample_time);
			
			// next d value is 
			new_d_value = s(new_s_value);
			
			// save new values if not exceeding s range
			if (new_s_value <= s_values[count]) {
				
				new_s_values.push_back(new_s_value);
				new_d_values.push_back(new_d_value);
				new_v_values.push_back(new_v_value);
				
			} else {
				
				bDone = true;
				
			}
			
		}
		
	}
	
	Trajectory::s_values = new_s_values;
	Trajectory::d_values = new_d_values;
	Trajectory::v_values = new_v_values;
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "  max_speed_change: " << max_speed_change << endl;
		cout << "  Trajectory::s_values: " << createDoubleVectorString(Trajectory::s_values);
		cout << "  Trajectory::d_values: " << createDoubleVectorString(Trajectory::d_values);
		cout << "  Trajectory::v_values: " << createDoubleVectorString(Trajectory::v_values);
		cout << "--- TRAJECTORY: set - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
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
vector<double> Trajectory::calculate_d_from_lane(const vector<unsigned int> &lane_values, const double &lane_width) {
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: calculate_d_from_lane - Start" << endl;
		cout << "  lane_values: " << createUnsignedIntegerVectorString(lane_values);
		cout << "  lane_width: " << lane_width << endl;
		
	}
	
	// define variables
	unsigned int count = 0;
	double d = 0;
	
	// initialize outputs
	vector<double> d_values;
	
	for (count = 0; count < lane_values.size(); count++) {
		
		d = (lane_width / 2) + (lane_width * ((double) lane_values[count] - 1));
		d_values.push_back(d);
		
	}
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "  d_values: " << createDoubleVectorString(d_values);
		cout << "--- TRAJECTORY: calculate_d_from_lane - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return d_values;
	
}

// transform from Frenet s,d coordinates to Cartesian x,y
vector<vector<double>> Trajectory::get_xy(const vector<double> &s_values, const vector<double> &d_values, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: get_xy - Start" << endl;
		cout << "  s_values: " << createDoubleVectorString(s_values);
		cout << "  d_values: " << createDoubleVectorString(d_values);
		//cout << "  maps_s: " << createDoubleVectorString(maps_s);
		//cout << "  maps_x: " << createDoubleVectorString(maps_x);
		//cout << "  maps_y: " << createDoubleVectorString(maps_y);
		
	}
	
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
		cout << "s: " << s << endl;
		cout << "d: " << d << endl;
		
		int prev_wp = -1;
		
		while ((s > maps_s[prev_wp + 1]) && (prev_wp < (int)(maps_s.size() - 1))) {
			
			prev_wp++;
			
		}
		cout << "prev_wp: " << prev_wp << endl;
		
		int wp2 = (prev_wp + 1) % maps_x.size();
		cout << "wp2: " << wp2 << endl;
		
		double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
		cout << "heading: ", heading << endl;
		// the x,y,s along the segment
		double seg_s = (s - maps_s[prev_wp]);
		cout << "seg_s: " << seg_s << endl;
		
		double seg_x = maps_x[prev_wp] + (seg_s * cos(heading));
		double seg_y = maps_y[prev_wp] + (seg_s * sin(heading));
		cout << "seg_x: " << seg_x << endl;
		cout << "seg_y: " << seg_y << endl;
		
		double perp_heading = heading - (M_PI / 2);
		cout << "perp_heading: " << perp_heading << endl;
		
		double x = seg_x + (d * cos(perp_heading));
		double y = seg_y + (d * sin(perp_heading));
		cout << "x: " << x << endl;
		cout << "y: " << y << endl;
		
		// save current point
		x_values.push_back(x);
		y_values.push_back(y);
		
	}
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "  x_values: " << createDoubleVectorString(x_values);
		cout << "  y_values: " << createDoubleVectorString(y_values);
		cout << "--- TRAJECTORY: get_xy - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return (vector<vector<double>>){x_values, y_values};
	
}