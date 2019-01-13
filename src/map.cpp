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
#include <vector>
#include <cmath>
#include "map.h"
#include "helper_functions.h"
#include "spline.h"

using std::vector;
using std::cout;
using std::endl;
using std::fabs;
using std::fmod;
using tk::spline;

// initialize map
void Map::init(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_INIT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: init - Start" << endl;
		cout << "  map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy: " << endl << createCarsVectorsString(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
		
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
		cout << "--- MAP: init - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}


// determine closest waypoint
vector<unsigned int> Map::closest_waypoint(const double &x, const double &y) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_CLOSEST_WAYPOINT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: closest_waypoint - Start" << endl;
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
		
		dist = distance(x, y, this->map_waypoints_x[count], this->map_waypoints_y[count]);
		distances.push_back(dist);
		
	}
	
	// generate index for closest waypoints
	closest_waypoints = sortDoubleVector(distances);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_CLOSEST_WAYPOINT) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  distances: " << endl << createDoubleVectorString(distances);
		cout << "  closest_waypoints: " << endl << createUnsignedIntegerVectorString(closest_waypoints);
		cout << "--- MAP: closest_waypoints - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return closest_waypoints;
	
}

// determine next waypoint
vector <unsigned int> Map::next_waypoint(const double &x, const double &y, const double &theta) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_NEXT_WAYPOINT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: next_waypoint - Start" << endl;
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
	closest_waypoints = this->closest_waypoint(x, y);
	
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
	if (bDISPLAY && bDISPLAY_MAP_NEXT_WAYPOINT) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  closest_waypoints: " << endl << createUnsignedIntegerVectorString(closest_waypoints);
		cout << "  next_waypoints: " << endl << createUnsignedIntegerVectorString(next_waypoints);
		cout << "--- MAP: next_waypoint - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return next_waypoints;
	
}

vector<double> Map::xy_to_frenet(const double &x, const double &y, const double &theta) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_XY_TO_FRENET) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: xy_to_frenet - Start" << endl;
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
	next_wps = this->next_waypoint(x, y, theta);
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
	d = distance(vector_x, vector_y, proj_x, proj_y);
	
	// see if d value is positive or negative by comparing it to a center point
	center_x = 1000 - this->map_waypoints_x[prev_wp];
	center_y = 2000 - this->map_waypoints_y[prev_wp];
	center_to_pos = distance(center_x, center_y, vector_x, vector_y);
	center_to_ref = distance(center_x, center_y, proj_x, proj_y);
	if (center_to_pos <= center_to_ref) {
		
		d *= -1;
		
	}
	
	// calculate s value as length from first waypoint to projection
	for (count = 0; count < prev_wp; count++) {
		
		s += distance(this->map_waypoints_x[count], this->map_waypoints_y[count], this->map_waypoints_x[count + 1], this->map_waypoints_y[count + 1]);
		
	}
	s += distance(0, 0, proj_x, proj_y);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_XY_TO_FRENET) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  s: " << s << endl;
		cout << "  d: " << d << endl;
		cout << "--- MAP: xy_to_frenet - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return (vector<double>){s, d};
}

vector<double> Map::frenet_to_xy(const double &s, const double &d) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_FRENET_TO_XY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAP: frenet_to_xy - Start" << endl;
		cout << "  s: " << s << endl;
		cout << "  d: " << d << endl;
		
	}
	
	// define variables
	double x = 0;
	double y = 0;
	
	// make sure we wrap around to the beginning at the end of the map
	s = fmod(s, MAX_TRACK_S);
	
	// use the splines to get a smooth path
	x = this->spline_x_s(s) + d * this->spline_dx_s(s);
	y = this->spline_y_s(s) + d * this->spline_dy_s(s);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_MAP_FRENET_TO_XY) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << y << endl;
		cout << "--- MAP: frenet_to_xy - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return (vector<double>){x, y};
	
}