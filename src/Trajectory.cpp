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
		cout << "  s_values: " << endl << createDoubleVectorString(s_values);
		cout << "  d_values: " << endl << createDoubleVectorString(d_values);
		cout << "  v_values: " << endl << createDoubleVectorString(v_values);
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
		cout << "  Trajectory::s_values: " << endl << createDoubleVectorString(Trajectory::s_values);
		cout << "  Trajectory::d_values: " << endl << createDoubleVectorString(Trajectory::d_values);
		cout << "  Trajectory::v_values: " << endl << createDoubleVectorString(Trajectory::v_values);
		cout << "--- TRAJECTORY: set - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// start trajectory with car state
void Trajectory::start_trajectory_with_car(Car &myCar, const double &max_acceleration_s, const double &sample_time) {
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: start_trajectory_with_car - Start" << endl;
		cout << "  myCar: " << endl << myCar.createString();
		cout << "  max_acceleration_s: " << max_acceleration_s << endl;
		cout << "  sample_time: " << sample_time << endl;
		
	}
	
	// define variables
	unsigned int count = 0;
	vector<double> new_s_values = Trajectory::s_values();
	vector<double> new_d_values = Trajectory::d_values();
	vector<double> new_v_values = Trajectory::v_values();
	double car_s = myCar.get_s();
	double car_d = myCar.get_d();
	double car_v = myCar.get_v();
	
	// translate trajectory to car state
	for (count = 0; count < new_s_values.size(); count++) {
		
		new_s_values[count] += car_s;
		
	}
	for (count = 0; count < new_d_values.size(); count++) {
		
		new_d_values[count] += car_d;
		
	}
	new_s_values.insert(new_s_values.begin(), car_s;
	new_d_values.insert(new_d_values.begin(), car_d);
	new_v_values.insert(new_v_values.begin(), car_v);
	
	// set new trajectory
	Trajectory::set(new_s_values, new_d_values, new_v_values, max_acceleration_s, sample_time);
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "  new_s_values: " << endl << createDoubleVectorString(new_s_values);
		cout << "  new_d_values: " << endl << createDoubleVectorString(new_d_values);
		cout << "  new_v_values: " << endl << createDoubleVectorString(new_v_values);
		cout << "--- TRAJECTORY: start_trajectory_with_car - End" << endl;
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
	if (bDISPLAY) {
		
		cout << "  d_values: " << endl << createDoubleVectorString(d_values);
		cout << "--- TRAJECTORY: calculate_d_from_lane - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return d_values;
	
}

// determine closest waypoint
int Trajectory::ClosestWaypoint(const double &x, const double &y, const vector<double> &maps_x, const vector<double> &maps_y) {
	// display message if required
	
	if (bDISPLAY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: ClosestWaypoint - Start" << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << y << endl;
		//cout << "  maps_x: " << endl << createDoubleVectorString(maps_x);
		//cout << "  maps_y: " << endl << createDoubleVectorString(maps_y);
		
	}
	
	double closestLen = 100000; //large number
	int closestWaypoint = 0;
	
	for(int i = 0; i < maps_x.size(); i++) {
		
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		
		if(dist < closestLen) {
			
			closestLen = dist;
			closestWaypoint = i;
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "  closestWaypoint: " << closestWaypoint << endl;
		cout << "--- TRAJECTORY: ClosestWaypoint - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return closestWaypoint;
	
}

// determine next waypoint
int Trajectory::NextWaypoint(const double &x, const double &y, const double &theta, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	if (bDISPLAY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: NextWaypoint - Start" << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << y << endl;
		cout << "  theta: " << theta << endl;
		//cout << "  maps_x: " << endl << createDoubleVectorString(maps_x);
		//cout << "  maps_y: " << endl << createDoubleVectorString(maps_y);
		
	}
	
	int closestWaypoint = Trajectory::ClosestWaypoint(x, y, maps_x, maps_y);
	
	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];
	
	double heading = atan2((map_y - y),(map_x - x));
	
	double angle = fabs(theta - heading);
	angle = min((2 * M_PI) - angle, angle);
	
	if(angle > (M_PI / 4)) {
		
		closestWaypoint++;
		
		if (closestWaypoint == maps_x.size()) {
			
			closestWaypoint = 0;
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "  closestWaypoint: " << closestWaypoint << endl;
		cout << "--- TRAJECTORY: NextWaypoint - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return closestWaypoint;
	
}

// transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<vector<double>> Trajectory::getFrenet(const vector<double> &x_values, const vector<double> &y_values, const vector<double> &theta_values, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	if (bDISPLAY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: getFrenet - Start" << endl;
		cout << "  x_values: " << endl << createDoubleVectorString(x_values);
		cout << "  y_values: " << endl << createDoubleVectorString(y_values);
		cout << "  theta_values: " << endl << createDoubleVectorString(theta_values);
		//cout << "  maps_x: " << endl << createDoubleVectorString(maps_x);
		//cout << "  maps_y: " << endl << createDoubleVectorString(maps_y);
		
	}
	
	// define variables
	unsigned int count = 0;
	double x = 0;
	double y = 0;
	double theta = 0;
	
	// initialize outputs
	vector<double> s_values;
	vector<double> d_values;
	
	// process all points
	for (count = 0; count < x_values.size(); count++) {
		
		x = x_values[count];
		y = y_values[count];
		theta = theta_values[count];
		
		int next_wp = Trajectory::NextWaypoint(x, y, theta, maps_x,maps_y);
		
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
	if (bDISPLAY) {
		
		cout << "  s_values: " << endl << createDoubleVectorString(s_values);
		cout << "  d_values: " << endl << createDoubleVectorString(d_values);
		cout << "--- TRAJECTORY: getFrenet - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return (vector<vector<double>>){s_values, d_values};
	
}

// transform from Frenet s,d coordinates to Cartesian x,y
vector<vector<double>> Trajectory::get_xy(const vector<double> &s_values, const vector<double> &d_values, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: get_xy - Start" << endl;
		cout << "  s_values: " << endl << createDoubleVectorString(s_values);
		cout << "  d_values: " << endl << createDoubleVectorString(d_values);
		//cout << "  maps_s: " << endl << createDoubleVectorString(maps_s);
		//cout << "  maps_x: " << endl << createDoubleVectorString(maps_x);
		//cout << "  maps_y: " << endl << createDoubleVectorString(maps_y);
		
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
	if (bDISPLAY) {
		
		cout << "  x_values: " << endl << createDoubleVectorString(x_values);
		cout << "  y_values: " << endl << createDoubleVectorString(y_values);
		cout << "--- TRAJECTORY: get_xy - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return (vector<vector<double>>){x_values, y_values};
	
}