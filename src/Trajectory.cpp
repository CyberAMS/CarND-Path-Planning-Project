/*
 * Trajectory.cpp
 *
 * Trajectory class
 *
 * Created on 01/04/2019
 * Author: Andre Strobel
 *
 */

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "helper_functions.h"
#include "Trajectory.h"
#include "Car.h"
#include "Path.h"
#include "spline.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;
using std::min;
using std::max;
using std::pow;
using std::sqrt;
using std::fabs;

// init trajectory
void Trajectory::init(Car myCar, Path myPreviousPath, const double &target_speed, const unsigned int &previous_path_steps, const double &sample_time, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_INIT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: init - Start" << endl;
		cout << "  myCar: " << endl << myCar.createString();
		cout << "  myPreviousPath: " << endl << myPreviousPath.createString();
		cout << "  target_speed: " << target_speed << endl;
		cout << "  previous_path_steps: " << previous_path_steps << endl;
		cout << "  sample_time: " << sample_time << endl;
		//cout << "  maps_s, maps_x, maps_y: " << endl << createDoubleVectorsString(vector<vector<double>>{maps_s, maps_x, maps_y});
	}
	
	// define variables
	vector<double> previous_x (2, 0);
	vector<double> previous_y (2, 0);
	double last_v = 0;
	double last_s = 0;
	double last_d = 0;
	vector<double> previous_path_x = myPreviousPath.get_x();
	vector<double> previous_path_y = myPreviousPath.get_y();
	unsigned int count = 0;
	vector<double> new_x_values;
	vector<double> new_y_values;
	double last_theta = 0;
	vector<vector<double>> last_sd;
	vector<vector<double>> new_xy_values;
	
	// determine initialization values
	if (previous_path_x.size() >= max((unsigned int)1, previous_path_steps)) {
		// enough previous path segments available
		
		// take current car position to determine angle and velocity of first previous path position
		previous_x[1] = myCar.get_x();
		previous_y[1] = myCar.get_y();
		last_s = myCar.get_s();
		
		// add short sequence of previous path positions starting with second previous path position
		for (count = 0; count < min((unsigned int)previous_path_x.size(), previous_path_steps); count++) {
			
			// take last previous path position to determine angle of next previous path position
			previous_x[0] = previous_x[1];
			previous_y[0] = previous_y[1];
			
			// add path xy to trajectory
			previous_x[1] = previous_path_x[count];
			previous_y[1] = previous_path_y[count];
			new_x_values.push_back(previous_x[1]);
			new_y_values.push_back(previous_y[1]);
			
			// remember path end sdv
			last_theta = atan2((previous_y[1] - previous_y[0]), (previous_x[1] - previous_x[0]));
			last_sd = Trajectory::getFrenet((vector<double>){previous_x[1]}, (vector<double>){previous_y[1]}, last_theta, maps_x, maps_y);
			last_v = distance(previous_x[0], previous_y[0], previous_x[1], previous_y[1]) / sample_time;
			if (last_v > 25) {
				cout << "********** CHECK **********" << endl;
			}
			cout << "previous_x[0], previous_y[0], previous_x[1], previous_y[1], last_theta, last_s, last_sd[0][0], last_v: " << previous_x[0] << "\t" << previous_y[0] << "\t" << previous_x[1] << "\t" << previous_y[1] << "\t" << last_theta << "\t" << last_s << "\t" << last_sd[0][0] << "\t" << last_v << endl;
			last_s = last_sd[0][0];
			last_d = last_sd[1][0];
			
		}
		
	} else {
		
		// predict car position in next step
		last_theta = myCar.get_theta();
		last_v = myCar.get_v();
		if (last_v == 0) {
			
			last_v = target_speed;
			
		}
		last_s = myCar.get_s() + (last_v * sample_time);
		last_d = myCar.get_d();
		
		// add path xy to trajectory
		new_xy_values = Trajectory::get_xy({last_s}, {last_d}, maps_s, maps_x, maps_y);
		new_x_values = new_xy_values[0];
		new_y_values = new_xy_values[1];
		
	}
	
	// initialize xy trajectory
	Trajectory::x_values = new_x_values;
	Trajectory::y_values = new_y_values;
	
	// initialize new trajectory segment with sdv
	Trajectory::s_values = (vector<double>){last_s};
	Trajectory::d_values = (vector<double>){last_d};
	Trajectory::v_values = (vector<double>){last_v};
	Trajectory::connect_theta = last_theta;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_INIT) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  Trajectory::x_values, Trajectory::Y_values: " << endl << createDoubleVectorsString(vector<vector<double>>{Trajectory::x_values, Trajectory::y_values});
		cout << "  Trajectory::s_values, Trajectory::d_values, Trajectory::v_values: " << endl << createDoubleVectorsString(vector<vector<double>>{Trajectory::s_values, Trajectory::d_values, Trajectory::v_values});
		cout << "  Trajectory::connect_theta: " << connect_theta << endl;
		cout << "--- TRAJECTORY: init - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// add new trajectory segment
void Trajectory::add(const unsigned int &current_lane, const double &current_speed, const unsigned int &target_lane, const double &target_speed, const double &lane_width, const double &max_acceleration_s, const double &max_acceleration_d, const double &max_waypoint_distance, const double &back_distance, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADD) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: add - Start" << endl;
		cout << "  current_lane: " << current_lane << endl;
		cout << "  current_speed: " << current_speed << endl;
		cout << "  target_lane: " << target_lane << endl;
		cout << "  target_speed: " << target_speed << endl;
		cout << "  lane_width: " << lane_width << endl;
		cout << "  max_acceleration_s: " << max_acceleration_s << endl;
		cout << "  max_acceleration_d: " << max_acceleration_d << endl;
		cout << "  max_waypoint_distance: " << max_waypoint_distance << endl;
		cout << "  back_distance: " << back_distance << endl;
		//cout << "  maps_x, maps_y: " << endl << createDoubleVectorsString(vector<vector<double>>{maps_x, maps_y});
		
	}
	
	// define variables
	double min_waypoint_distance_s = 0;
	vector<double> lane_distances;
	double lateral_change = 0;
	double min_lateral_change_time = 0;
	double min_waypoint_distance_d = 0;
	double min_waypoint_distance_calc = 0;
	unsigned int num_found = 0;
	vector<double> waypoints_x;
	vector<double> waypoints_y;
	vector<unsigned int> next_waypoints;
	unsigned int count1 = 0;
	vector<vector<double>> next_sd;
	unsigned int count2 = 0;
	
	// determine required distance to first waypoint
	min_waypoint_distance_s = pow((target_speed - current_speed), 2) / max_acceleration_s; // distance traveled in s to reach target speed
	lane_distances = Trajectory::calculate_d_from_lane((vector<unsigned int>){current_lane, target_lane}, lane_width);
	lateral_change = fabs(lane_distances[1] - lane_distances[0]); // distance from target lane to current lane
	min_lateral_change_time = sqrt(2 * lateral_change / max_acceleration_d); // time needed to change lanes
	min_waypoint_distance_d = current_speed * min_lateral_change_time; // distance traveled in s when changing lanes
	min_waypoint_distance_calc = max(min_waypoint_distance_s, min_waypoint_distance_d);
	
	// get next waypoint
	next_waypoints = Trajectory::NextWaypoint(Trajectory::x_values.back(), Trajectory::y_values.back(), Trajectory::connect_theta, maps_x, maps_y);
	
	// determine first waypoint
	for (count1 = 0; count1 < next_waypoints.size(); count1++) {
		
		if (distance(Trajectory::x_values.back(), Trajectory::y_values.back(), maps_x[next_waypoints[count1]], maps_y[next_waypoints[count1]]) < min(min_waypoint_distance_calc, max_waypoint_distance)) {
			
			break; // for loop
			
		}
		
	}
	waypoints_x.push_back(maps_x[next_waypoints[count1]]);
	waypoints_y.push_back(maps_y[next_waypoints[count1]]);
	next_sd = Trajectory::getFrenet((vector<double>){waypoints_x.back()}, (vector<double>){waypoints_y.back()}, Trajectory::connect_theta, maps_x, maps_y);
	Trajectory::s_values.push_back(next_sd[0][0]);
	Trajectory::d_values.push_back(lane_distances[1]);
	Trajectory::v_values.push_back(target_speed);
	
	// determine second waypoint (don't reset count for quicker search)
	for (count2 = count1; count2 < next_waypoints.size(); count2++) {
		
		if (distance(waypoints_x.back(), waypoints_y.back(), maps_x[next_waypoints[count2]], maps_y[next_waypoints[count2]]) < min(back_distance, max_waypoint_distance)) {
			
			break; // for loop
			
		}
		
	}
	waypoints_x.push_back(maps_x[next_waypoints[count2]]);
	waypoints_y.push_back(maps_y[next_waypoints[count2]]);
	next_sd = Trajectory::getFrenet((vector<double>){waypoints_x.back()}, (vector<double>){waypoints_y.back()}, Trajectory::connect_theta, maps_x, maps_y);
	Trajectory::s_values.push_back(next_sd[0][0]);
	Trajectory::d_values.push_back(lane_distances[1]);
	Trajectory::v_values.push_back(target_speed);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADD) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  min_waypoint_distance_s: " << min_waypoint_distance_s << endl;
		cout << "  lane_distances: " << endl << createDoubleVectorString(lane_distances);
		cout << "  lateral_change: " << lateral_change << endl;
		cout << "  min_lateral_change_time: " << min_lateral_change_time << endl;
		cout << "  min_waypoint_distance_d: " << min_waypoint_distance_d << endl;
		cout << "  min_waypoint_distance_calc: " << min_waypoint_distance_calc << endl;
		//cout << "  next_waypoints: " << endl << createUnsignedIntegerVectorString(next_waypoints);
		cout << "  waypoints_x, waypoints_y: " << endl << createDoubleVectorsString(vector<vector<double>>{waypoints_x, waypoints_y});
		cout << "  Trajectory::s_values, Trajectory::d_values, Trajectory::v_values: " << endl << createDoubleVectorsString(vector<vector<double>>{Trajectory::s_values, Trajectory::d_values, Trajectory::v_values});
		cout << "--- TRAJECTORY: add - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// add new trajectory segment with fixed spacing
void Trajectory::add_fixed(const unsigned int &target_lane, const double &target_speed, const double &lane_width, const vector<double> &fixed_s_distances) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADD_FIXED) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: add_fixed - Start" << endl;
		cout << "  target_lane: " << target_lane << endl;
		cout << "  target_speed: " << target_speed << endl;
		cout << "  lane_width: " << lane_width << endl;
		cout << "  fixed_s_distances: " << endl << createDoubleVectorString(fixed_s_distances);
		
	}
	
	// define variables
	vector<double> lane_distances;
	
	// determine lane positions
	lane_distances = Trajectory::calculate_d_from_lane((vector<unsigned int>){target_lane}, lane_width);
	
	// determine first waypoint
	Trajectory::s_values.push_back((Trajectory::s_values.back() + fixed_s_distances[0]));
	Trajectory::d_values.push_back(lane_distances[0]);
	Trajectory::v_values.push_back(target_speed);
	
	// determine second waypoint
	Trajectory::s_values.push_back((Trajectory::s_values.back() + (fixed_s_distances[1] - fixed_s_distances[0])));
	Trajectory::d_values.push_back(lane_distances[0]);
	Trajectory::v_values.push_back(target_speed);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADD_FIXED) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  lane_distances: " << endl << createDoubleVectorString(lane_distances);
		cout << "  Trajectory::s_values, Trajectory::d_values, Trajectory::v_values: " << endl << createDoubleVectorsString(vector<vector<double>>{Trajectory::s_values, Trajectory::d_values, Trajectory::v_values});
		cout << "--- TRAJECTORY: add_fixed - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// calculate full trajectory
void Trajectory::calculate(Car myCar, const double &back_distance, const double &sample_time, const double &max_acceleration_s, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_CALCULATE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: calculate - Start" << endl;
		cout << "  back_distance: " << back_distance << endl;
		cout << "  sample_time: " << sample_time << endl;
		//cout << "  maps_s, maps_x, maps_y: " << endl << createDoubleVectorsString(vector<vector<double>>{maps_s, maps_x, maps_y});
		
	}
	
	// define variables
	double start_s = Trajectory::s_values[0];
	double start_v = Trajectory::v_values[0];
	double start_x = Trajectory::x_values.back(); // !!!! XXX TODO maybe one point earlier or later?
	double start_y = Trajectory::y_values.back(); // !!!! XXX TODO maybe one point earlier or later?
	vector<vector<double>> back_sd;
	double back_s = 0;
	double back_v = 0;
	double back_x = 0;
	double back_y = 0;
	vector<double> spline_s_values;
	vector<double> spline_v_values;
	vector<double> spline_x_values;
	vector<double> spline_y_values;
	vector<vector<double>> new_xy_values;
	vector<double> new_x_values;
	vector<double> new_y_values;
	unsigned int count = 0;
	tk::spline s_v;
	tk::spline s_x;
	tk::spline s_y;
	double next_s_value = 0;
	double new_v_value = 0;
	double new_x_value = 0;
	double new_y_value = 0;
	bool bAccelerate = false;
	bool bDecelerate = false;
	
	// determine start of new spline trajectory segment with previous end
	back_v = start_v;
	back_x = start_x - (cos(Trajectory::connect_theta) * back_distance);
	back_y = start_y - (sin(Trajectory::connect_theta) * back_distance);
	back_sd = Trajectory::getFrenet((vector<double>){back_x}, (vector<double>){back_y}, Trajectory::connect_theta, maps_x, maps_y);
	back_s = back_sd[0][0];
	spline_s_values.push_back(back_s);
	spline_v_values.push_back(back_v);
	spline_x_values.push_back(back_x);
	spline_y_values.push_back(back_y);
	
	// second point of new spline trajectory segment is previous end
	spline_s_values.push_back(start_s);
	spline_v_values.push_back(start_v);
	spline_x_values.push_back(start_x);
	spline_y_values.push_back(start_y);
	
	// determine new spline trajectory segments
	new_xy_values = Trajectory::get_xy(Trajectory::s_values, Trajectory::d_values, maps_s, maps_x, maps_y);
	new_x_values = new_xy_values[0];
	new_y_values = new_xy_values[1];
	for (count = 1; count < new_x_values.size(); count++) {
		
		spline_s_values.push_back(Trajectory::s_values[count]);
		spline_v_values.push_back(Trajectory::v_values[count]);
		spline_x_values.push_back(new_x_values[count]);
		spline_y_values.push_back(new_y_values[count]);
		
	}
	
	// create new spline trajectory
	s_v.set_points(spline_s_values, spline_v_values);
	s_x.set_points(spline_s_values, spline_x_values);
	s_y.set_points(spline_s_values, spline_y_values);
	
	// define trajectory segment start
	next_s_value = start_s + (start_v * sample_time);
	
	// determine trajectory segment points
	while (next_s_value < Trajectory::s_values.back()) {
		
		// LOTS OF SIMPLIFICATIONS BELOW XXXXXXXXX !!!!!!!!!!!!!!!! TODO
		
		// next speed value is old speed value plus maximum delta to achieve next target value
		new_v_value = 48 * MPH_TO_MS; // (max(min(s_v(next_s_value), 25.0), 10.0));
		new_x_value = s_x(next_s_value);
		new_y_value = s_y(next_s_value);
		
		cout << "next_s_value, new_v_value, new_x_value, new_y_value: " << next_s_value << "\t" << new_v_value << "\t" << new_x_value << "\t" << new_y_value << endl;
		
		// add current trajectory segment points to final trajectory
		// !!!!! XXXX TODO Note sure why the first x value here can be less than the last x value of the previous path - should be more (one step more)
		Trajectory::x_values.push_back(new_x_value);
		Trajectory::y_values.push_back(new_y_value);
	
		// check what to do in case of standstill
		bAccelerate = false;
		bDecelerate = false;
		if (0 == 0) {
			
			bAccelerate = true;
			
		} else if (0 == 1) {
			
			bDecelerate = true;
			
		}
		
		// determine next s value based on speed of current trajectory segment point
		if ((new_v_value == 0) && (bAccelerate)) {
			
			next_s_value += (max_acceleration_s / 50); //* sample_time * sample_time);
			
		} else if ((new_v_value == 0) && (bDecelerate)) {
			
			next_s_value -= (max_acceleration_s * sample_time * sample_time);
			
		} else {
			
			// !!!!!!!! XXXXXXXXX TODO Not sure why the velocity is > 50 m/s - is the myCar value in mph?
			next_s_value += (new_v_value * sample_time);
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_CALCULATE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  spline_s_values, spline_v_values, spline_x_values, spline_y_values: " << endl << createDoubleVectorsString(vector<vector<double>>{spline_s_values, spline_v_values, spline_x_values, spline_y_values});
		cout << "  Trajectory::x_values, Trajectory::y_values: " << endl << createDoubleVectorsString(vector<vector<double>>{Trajectory::x_values, Trajectory::y_values});
		cout << "--- TRAJECTORY: calculate - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// get trajectory x values
vector<double> Trajectory::get_x() {
	
	return Trajectory::x_values;
	
}

// get trajectory y values
vector<double> Trajectory::get_y() {
	
	return Trajectory::y_values;
	
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
	if (bDISPLAY && bDISPLAY_TRAJECTORY_CALCULATE_D_FROM_LANE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: calculate_d_from_lane - Start" << endl;
		cout << "  lane_values: " << endl << createUnsignedIntegerVectorString(lane_values);
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
	if (bDISPLAY && bDISPLAY_TRAJECTORY_CALCULATE_D_FROM_LANE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  d_values: " << endl << createDoubleVectorString(d_values);
		cout << "--- TRAJECTORY: calculate_d_from_lane - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return d_values;
	
}

// estimate lane number for lane based on distance from center lane
vector<unsigned int> Trajectory::estimate_lanes(const vector<double> &d_values, const vector<unsigned int> &lanes, const double &lane_width) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ESTIMATE_LANES) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: estimate_lanes - Start" << endl;
		cout << "  d_values: " << endl << createDoubleVectorString(d_values);
		cout << "  lanes: " << endl << createUnsignedIntegerVectorString(lanes);
		cout << "  lane_width: " << lane_width << endl;
		
	}
	
	// define variables
	vector<double> lane_centers;
	unsigned int count_d = 0;
	double min_distance = 0;
	unsigned int best_lane = 0;
	unsigned int count_l = 0;
	double lane_distance = 0;
	vector<unsigned int> estimated_lanes;
	
	// get center for all lanes
	lane_centers = Trajectory::calculate_d_from_lane(lanes, lane_width);
	
	// determine lane for all given distance values
	for (count_d = 0; count_d < d_values.size(); count_d++) {
		
		// reset minimum distance and best lane
		min_distance = std::numeric_limits<double>::max();
		best_lane = 0;
		
		// check all lanes for minimum distance to current distance value
		for (count_l = 0; count_l < lane_centers.size(); count_l++) {
			
			lane_distance = fabs(d_values[count_d] - lane_centers[count_l]);
			
			if (lane_distance < min_distance) {
				
				min_distance = lane_distance;
				best_lane = lanes[count_l];
				
			}
			
		}
		
		// save best lane number for current distance value
		estimated_lanes.push_back(best_lane);
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_CALCULATE_D_FROM_LANE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  estimated_lanes: " << endl << createUnsignedIntegerVectorString(estimated_lanes);
		cout << "--- TRAJECTORY: estimate_lanes - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return estimated_lanes;
	
}

// determine closest waypoint
vector<unsigned int> Trajectory::ClosestWaypoint(const double &x, const double &y, const vector<double> &maps_x, const vector<double> &maps_y) {
	// display message if required
	
	if (bDISPLAY && bDISPLAY_TRAJECTORY_CLOSESTWAYPOINT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: ClosestWaypoint - Start" << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << y << endl;
		//cout << "  maps_x, maps_y: " << endl << createDoubleVectorsString(vector<vector<double>>{maps_x, maps_y});
		
	}
	
	// define variables
	unsigned int count = 0;
	double dist = 0;
	vector<double> distances;
	vector<unsigned int> closestWaypoint;
	
	for(count = 0; count < maps_x.size(); count++) {
		
		dist = distance(x, y, maps_x[count], maps_y[count]);
		distances.push_back(dist);
		
	}
	
	closestWaypoint = sortDoubleVector(distances);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_CLOSESTWAYPOINT) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  distances: " << endl << createDoubleVectorString(distances);
		cout << "  closestWaypoint: " << endl << createUnsignedIntegerVectorString(closestWaypoint);
		cout << "--- TRAJECTORY: ClosestWaypoint - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return closestWaypoint;
	
}

// determine next waypoint
vector <unsigned int> Trajectory::NextWaypoint(const double &x, const double &y, const double &theta, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	if (bDISPLAY && bDISPLAY_TRAJECTORY_NEXTWAYPOINT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: NextWaypoint - Start" << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << y << endl;
		cout << "  theta: " << theta << endl;
		//cout << "  maps_x, maps_y: " << endl << createDoubleVectorsString(vector<vector<double>>{maps_x, maps_y});
		
	}
	
	// define variables
	vector<unsigned int> closestWaypoint;
	unsigned int count = 0;
	unsigned int current_waypoint = 0;
	vector<unsigned int> nextWaypoint;
	double heading = 0;
	double angle = 0;
	
	closestWaypoint = Trajectory::ClosestWaypoint(x, y, maps_x, maps_y);
	
	for (count = 0; count < closestWaypoint.size(); count++) {
		
		current_waypoint = closestWaypoint[count];
		
		heading = atan2((maps_y[current_waypoint] - y), (maps_x[current_waypoint] - x));
		angle = fabs(theta - heading);
		angle = min((2 * M_PI) - angle, angle);
		
		if(angle <= (M_PI / 4)) {
			
			nextWaypoint.push_back(current_waypoint);
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_NEXTWAYPOINT) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  closestWaypoint: " << endl << createUnsignedIntegerVectorString(closestWaypoint);
		cout << "  nextWaypoint: " << endl << createUnsignedIntegerVectorString(nextWaypoint);
		cout << "--- TRAJECTORY: NextWaypoint - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return nextWaypoint;
	
}

// transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<vector<double>> Trajectory::getFrenet(const vector<double> &x_values, const vector<double> &y_values, const double &theta, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	if (bDISPLAY && bDISPLAY_TRAJECTORY_GETFRENET) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: getFrenet - Start" << endl;
		cout << "  x_values, y_values: " << endl << createDoubleVectorsString(vector<vector<double>>{x_values, y_values});
		cout << "  theta: " << theta << endl;
		//cout << "  maps_x, maps_y: " << endl << createDoubleVectorsString(vector<vector<double>>{maps_x, maps_y});
		
	}
	
	// define variables
	unsigned int count = 0;
	double x = 0;
	double y = 0;
	
	// initialize outputs
	vector<double> s_values;
	vector<double> d_values;
	
	// process all points
	for (count = 0; count < x_values.size(); count++) {
		
		x = x_values[count];
		y = y_values[count];
		
		vector<unsigned int> next_wps = Trajectory::NextWaypoint(x, y, theta, maps_x,maps_y);
		
		int next_wp = (int)next_wps[0];
		
		int prev_wp;
		prev_wp = next_wp - 1;
		
		if(next_wp == 0) {
			
			prev_wp  = maps_x.size() - 1;
			
		}
		
		double n_x = maps_x[next_wp] - maps_x[prev_wp];
		double n_y = maps_y[next_wp] - maps_y[prev_wp];
		double x_x = x - maps_x[prev_wp];
		double x_y = y - maps_y[prev_wp];
		
		// find the projection of x onto n
		double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
		double proj_x = proj_norm * n_x;
		double proj_y = proj_norm * n_y;
		
		double frenet_d = distance(x_x, x_y, proj_x, proj_y);
		
		//see if d value is positive or negative by comparing it to a center point
		
		double center_x = 1000 - maps_x[prev_wp];
		double center_y = 2000 - maps_y[prev_wp];
		double centerToPos = distance(center_x, center_y, x_x, x_y);
		double centerToRef = distance(center_x, center_y, proj_x, proj_y);
		
		if(centerToPos <= centerToRef) {
			
			frenet_d *= -1;
			
		}
		
		// calculate s value
		double frenet_s = 0;
		for(int i = 0; i < prev_wp; i++) {
			
			frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
			
		}
		
		frenet_s += distance(0, 0, proj_x, proj_y);
		
		// save current point
		s_values.push_back(frenet_s);
		d_values.push_back(frenet_d);
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_GETFRENET) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  s_values, d_values: " << endl << createDoubleVectorsString(vector<vector<double>>{s_values, d_values});
		cout << "--- TRAJECTORY: getFrenet - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return (vector<vector<double>>){s_values, d_values};
	
}

// transform from Frenet s,d coordinates to Cartesian x,y
vector<vector<double>> Trajectory::get_xy(const vector<double> &s_values, const vector<double> &d_values, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_GET_XY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: get_xy - Start" << endl;
		cout << "  s_values, d_values: " << endl << createDoubleVectorsString(vector<vector<double>>{s_values, d_values});
		//cout << "  maps_s, maps_x, maps_y: " << endl << createDoubleVectorsString(vector<vector<double>>{maps_s, maps_x, maps_y});
		
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
		
		int prev_wp = -1;
		
		while ((s >= maps_s[prev_wp + 1]) && (prev_wp < (int)(maps_s.size() - 1))) {
			// changed to "s >=" from "s >"
			
			prev_wp++;
			
		}
		
		int wp2 = (prev_wp + 1) % maps_x.size();
		
		double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
		// the x,y,s along the segment
		double seg_s = (s - maps_s[prev_wp]);
		
		double seg_x = maps_x[prev_wp] + (seg_s * cos(heading));
		double seg_y = maps_y[prev_wp] + (seg_s * sin(heading));
		
		double perp_heading = heading - (M_PI / 2);
		
		double x = seg_x + (d * cos(perp_heading));
		double y = seg_y + (d * sin(perp_heading));
		
		// save current point
		x_values.push_back(x);
		y_values.push_back(y);
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_GET_XY) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  x_values, y_values: " << endl << createDoubleVectorsString(vector<vector<double>>{x_values, y_values});
		cout << "--- TRAJECTORY: get_xy - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return (vector<vector<double>>){x_values, y_values};
	
}