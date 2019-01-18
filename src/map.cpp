/*
 * Map.cpp
 *
 * Map class
 *
 * Created on 01/12/2019
 * Author: Andre Strobel
 *
 */

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "Map.h"
#include "helper_functions.h"
#include "spline.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;
using std::fabs;
using std::min;
using std::fmod;

// initialize map
void Map::Init(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_INIT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: Init - Start" << endl;
		cout << "  map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy: " << endl << CreateDoubleVectorsString((vector<vector<double>>){map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy});
		
	}
	
	// store waypoint data
	this->map_waypoints_x = map_waypoints_x;
	this->map_waypoints_y = map_waypoints_y;
	this->map_waypoints_s = map_waypoints_s;
	this->map_waypoints_dx = map_waypoints_dx;
	this->map_waypoints_dy = map_waypoints_dy;
	
	// create waypoint splines
	this->spline_x_s.set_points(this->map_waypoints_s, this->map_waypoints_x);
	this->spline_y_s.set_points(this->map_waypoints_s, this->map_waypoints_y);
	this->spline_dx_s.set_points(this->map_waypoints_s, this->map_waypoints_dx);
	this->spline_dy_s.set_points(this->map_waypoints_s, this->map_waypoints_dy);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_INIT) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "--- MAP: Init - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// convert xy to Frenet coordinates
vector<double> Map::Xy2Frenet(const double &x, const double &y, const double &theta) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_XY2FRENET) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: Xy2Frenet - Start" << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << y << endl;
		cout << "  theta: " << theta << endl;
		
	}
	
	// define variables
	vector<unsigned int> next_wps;
	unsigned int next_wp = 0;
	unsigned int prev_wp = 0;
	double normal_x = 0;
	double normal_y = 0;
	double vector_x = 0;
	double vector_y = 0;
	double proj_norm = 0;
	double proj_x = 0;
	double proj_y = 0;
	double d = 0;
	double center_x = 0;
	double center_y = 0;
	double center_to_pos = 0;
	double center_to_ref = 0;
	unsigned int count = 0;
	double s = 0;
	
	// get next waypoint
	next_wps = this->NextWaypoint(x, y, theta);
	next_wp = next_wps[0];
	
	// get previous waypoint
	if (next_wp == 0) {
		
		prev_wp = this->map_waypoints_x.size() - 1;
		
	} else {
		
		prev_wp = next_wp - 1;
		
	}
	
	// calculate normal s direction and vector to xy
	normal_x = this->map_waypoints_x[next_wp] - this->map_waypoints_x[prev_wp];
	normal_y = this->map_waypoints_y[next_wp] - this->map_waypoints_y[prev_wp];
	vector_x = x - this->map_waypoints_x[prev_wp];
	vector_y = y - this->map_waypoints_y[prev_wp];
	
	// find the projection of vector xy onto normal s direction
	proj_norm = ((vector_x * normal_x) + (vector_y * normal_y)) / ((normal_x * normal_x) + (normal_y * normal_y));
	proj_x = proj_norm * normal_x;
	proj_y = proj_norm * normal_y;
	
	// calculate d as distance to projection
	d = Distance(vector_x, vector_y, proj_x, proj_y);
	
	// see if d value is positive or negative by comparing it to a center point
	center_x = 1000 - this->map_waypoints_x[prev_wp];
	center_y = 2000 - this->map_waypoints_y[prev_wp];
	center_to_pos = Distance(center_x, center_y, vector_x, vector_y);
	center_to_ref = Distance(center_x, center_y, proj_x, proj_y);
	if (center_to_pos <= center_to_ref) {
		
		d *= -1;
		
	}
	
	// calculate s value as length from first waypoint to projection
	for (count = 0; count < prev_wp; count++) {
		
		s += Distance(this->map_waypoints_x[count], this->map_waypoints_y[count], this->map_waypoints_x[count + 1], this->map_waypoints_y[count + 1]);
		
	}
	s += Distance(0, 0, proj_x, proj_y);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_XY2FRENET) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  s: " << s << endl;
		cout << "  d: " << d << endl;
		cout << "--- MAP: Xy2Frenet - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return (vector<double>){s, d};
}

// convert Frenet to xy coordinates
vector<double> Map::Frenet2Xy(const double &s, const double &d) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_FRENET2XY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: Frenet2Xy - Start" << endl;
		cout << "  s: " << s << endl;
		cout << "  d: " << d << endl;
		
	}
	
	// define variables
	double clean_s = 0;
	
	// initialize outputs
	double x = 0;
	double y = 0;
	
	// make sure we wrap around to the beginning at the end of the map
	clean_s = fmod(s, MAX_TRACK_S);
	
	// use the splines to get a smooth path
	x = this->spline_x_s(clean_s) + d * this->spline_dx_s(clean_s);
	y = this->spline_y_s(clean_s) + d * this->spline_dy_s(clean_s);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_FRENET2XY) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << y << endl;
		cout << "--- MAP: Frenet2Xy - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return (vector<double>){x, y};
	
}
vector<vector<double>> Map::Frenet2Xy(const vector<double> &s_values, const vector<double> &d_values) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_FRENET2XY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: Frenet2Xy - Start" << endl;
		cout << "  s_values: " << endl << CreateDoubleVectorString(s_values);
		cout << "  d_values: " << endl << CreateDoubleVectorString(d_values);
		
	}
	
	// define variables
	unsigned long count = 0;
	vector<double> xy_value;
	
	// initialize outputs
	vector<double> x_values;
	vector<double> y_values;
	
	for (count = 0; count < s_values.size(); count++) {
		
		// convert current element from Frenet to xy coordinates
		xy_value = this->Frenet2Xy(s_values[count], d_values[count]);
		x_values.push_back(xy_value[0]);
		y_values.push_back(xy_value[1]);
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_FRENET2XY) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  x_values: " << endl << CreateDoubleVectorString(x_values);
		cout << "  y_values: " << endl << CreateDoubleVectorString(y_values);
		cout << "--- MAP: Frenet2Xy - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return (vector<vector<double>>){x_values, y_values};
	
}

// display Map object as string
string Map::CreateString() {
	
	//define variables
	string text = "";
	
	// add information about car to string
	text += DISPLAY_PREFIX + "map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy =\n" + CreateDoubleVectorsString(vector<vector<double>>{map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy});
	
	// return output
	return text;
	
}

// determine closest waypoint
vector<unsigned int> Map::ClosestWaypoint(const double &x, const double &y) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_CLOSESTWAYPOINT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: ClosestWaypoint - Start" << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << y << endl;
		
	}
	
	// define variables
	unsigned int count = 0;
	double dist = 0;
	vector<double> distances;
	vector<unsigned int> closest_waypoints;
	
	// calculate distances to all waypoints
	for(count = 0; count < this->map_waypoints_x.size(); count++) {
		
		dist = Distance(x, y, this->map_waypoints_x[count], this->map_waypoints_y[count]);
		distances.push_back(dist);
		
	}
	
	// generate index for closest waypoints
	closest_waypoints = SortDoubleVector(distances);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_CLOSESTWAYPOINT) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  distances: " << endl << CreateDoubleVectorString(distances);
		cout << "  closest_waypoints: " << endl << CreateUnsignedIntegerVectorString(closest_waypoints);
		cout << "--- MAP: ClosestWaypoints - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return closest_waypoints;
	
}

// determine next waypoint
vector <unsigned int> Map::NextWaypoint(const double &x, const double &y, const double &theta) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_NEXTWAYPOINT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: NextWaypoint - Start" << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << y << endl;
		cout << "  theta: " << theta << endl;
		
	}
	
	// define variables
	vector<unsigned int> closest_waypoints;
	unsigned int count = 0;
	unsigned int current_waypoint = 0;
	vector<unsigned int> next_waypoints;
	double heading = 0;
	double angle = 0;
	
	// get closest waypoints
	closest_waypoints = this->ClosestWaypoint(x, y);
	
	// check all closest waypoints for being valid
	for (count = 0; count < closest_waypoints.size(); count++) {
		
		current_waypoint = closest_waypoints[count];
		
		heading = atan2((this->map_waypoints_y[current_waypoint] - y), (this->map_waypoints_x[current_waypoint] - x));
		angle = fabs(theta - heading);
		angle = min((2 * M_PI) - angle, angle);
		
		// current waypoint is ahead
		if(angle <= (M_PI / 4)) {
			
			next_waypoints.push_back(current_waypoint);
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_NEXTWAYPOINT) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  closest_waypoints: " << endl << CreateUnsignedIntegerVectorString(closest_waypoints);
		cout << "  next_waypoints: " << endl << CreateUnsignedIntegerVectorString(next_waypoints);
		cout << "--- MAP: NextWaypoint - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return next_waypoints;
	
}