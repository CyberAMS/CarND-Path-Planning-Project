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

// TODO: Need to fix functions that use s to calculate distances and get stuck when MAX_S of map is reached

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
	clean_s = this->AssignS(s);
	
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

// assign correct s value considering loop track
double Map::AssignS(const double &s) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_ASSIGNS) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: AssignS - Start" << endl;
		cout << "  s: " << s << endl;
		
	}
	
	// initialize outputs
	double clean_s = 0.0;
	
	// calculate s value considering loop track
	clean_s = fmod(s, MAX_TRACK_S);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_ASSIGNS) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  clean_s: " << clean_s << endl;
		cout << "--- MAP: AssignS - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return clean_s;
	
}
vector<double> Map::AssignS(const vector<double> &s) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_ASSIGNS) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: AssignS - Start" << endl;
		cout << "  s: " << endl << CreateDoubleVectorString(s);
		
	}
	
	// define variables
	unsigned long count = 0;
	
	// initialize outputs
	vector<double> clean_s;
	
	// execute this for all elements
	for (count = 0; count < s.size(); count++) {
		
		clean_s.push_back(this->AssignS(s[count]));
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_ASSIGNS) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  clean_s: " << endl << CreateDoubleVectorString(clean_s);
		cout << "--- MAP: AssignS - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return clean_s;
	
}

// calculate absolute difference between two s values considering loop track
double Map::DeltaS(const double &s1, const double &s2) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_DELTAS) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: DeltaS - Start" << endl;
		cout << "  s1: " << s1 << endl;
		cout << "  s2: " << s2 << endl;
		
	}
	
	// TODO: remove this shortcut
	return (s1 - s2);
	
	// define variables
	double reference_s1 = 0.0;
	double reference_s2 = 0.0;
	
	// initialize outputs
	double delta_s = 0.0;
	
	// calculate values relative to reference
	reference_s1 = this->ReferenceS(s1, s2); // use s2 value as reference => this value is the delta
	// reference_s2 = this->ReferenceS(s2, s2); // use s2 value as reference => this value is 0
	
	// calculate delta
	delta_s = reference_s1 - reference_s2;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_DELTAS) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  reference_s1: " << reference_s1 << endl;
		cout << "  reference_s2: " << reference_s2 << endl;
		cout << "  delta_s: " << delta_s << endl;
		cout << "--- MAP: DeltaS - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return delta_s;
	
}
vector<double> Map::DeltaS(const vector<double> &s1, const vector<double> &s2) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_DELTAS) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: DeltaS - Start" << endl;
		cout << "  s1, s2: " << endl << CreateDoubleVectorsString((vector<vector<double>>){s1, s2});
		
	}
	
	// define variables
	unsigned long count = 0;
	
	// initialize outputs
	vector<double> delta_s;
	
	// execute this for all elements
	for (count = 0; count < s1.size(); count++) {
		
		delta_s.push_back(this->DeltaS(s1[count], s2[count]));
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_DELTAS) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  delta_s: " << endl << CreateDoubleVectorString(delta_s);
		cout << "--- MAP: DeltaS - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return delta_s;
	
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

// calculate s value relative to reference considering loop track
double Map::ReferenceS(const double &s, const double &s_reference) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_REFERENCES) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: ReferenceS - Start" << endl;
		cout << "  s: " << s << endl;
		cout << "  s_reference: " << s_reference << endl;
		
	}
	
	// define variables
	double clean_s = 0.0;
	double clean_s_reference = 0.0;
	double s_mid_point = 0.0;
	
	// initialize outputs
	double reference_s = 0.0;
	
	// clean inputs so they are in single loop coordinates
	clean_s = this->AssignS(s);
	clean_s_reference = this->AssignS(s_reference);
	
	// calculate point on the other side of the loop track
	s_mid_point = this->AssignS(clean_s_reference + (MAX_TRACK_S / 2));
	
	// check whether reference point is in first half of the loop
	if (clean_s_reference <= s_mid_point) {
		
		// check which segment of the loop applies
		if ((clean_s > clean_s_reference) && (clean_s <= s_mid_point)) {
			
			reference_s = clean_s - clean_s_reference;
			
		} else if ((clean_s > s_mid_point) && (clean_s <= MAX_TRACK_S)) {
			
			reference_s = (s_mid_point - MAX_TRACK_S) - clean_s_reference;
			
		} else if ((clean_s >= 0) && (clean_s <= clean_s_reference)) {
			
			reference_s = clean_s - clean_s_reference;
			
		}
		
	} else {
		
		// check which segment of the loop applies
		if ((clean_s > clean_s_reference) && (clean_s <= MAX_TRACK_S)) {
			
			reference_s = clean_s - clean_s_reference;
			
		} else if ((clean_s >= 0) && (clean_s <= s_mid_point)) {
			
			reference_s = (MAX_TRACK_S - clean_s_reference) + clean_s;
			
		} else if ((clean_s > s_mid_point) && (clean_s <= clean_s_reference)) {
			
			reference_s = clean_s_reference - clean_s;
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_REFERENCES) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  clean_s: " << clean_s << endl;
		cout << "  clean_s_reference: " << clean_s_reference << endl;
		cout << "  s_mid_point: " << s_mid_point << endl;
		cout << "  reference_s: " << reference_s << endl;
		cout << "--- MAP: ReferenceS - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return reference_s;
	
}
vector<double> Map::ReferenceS(const vector<double> &s, const vector<double> &s_reference) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_REFERENCES) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: ReferenceS - Start" << endl;
		cout << "  s, s_reference: " << endl << CreateDoubleVectorsString((vector<vector<double>>){s, s_reference});
		
	}
	
	// define variables
	unsigned long count = 0;
	
	// initialize outputs
	vector<double> reference_s;
	
	// execute this for all elements
	for (count = 0; count < s.size(); count++) {
		
		reference_s.push_back(this->ReferenceS(s[count], s_reference[count]));
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_REFERENCES) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  reference_s: " << endl << CreateDoubleVectorString(reference_s);
		cout << "--- MAP: ReferenceS - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return reference_s;
	
}