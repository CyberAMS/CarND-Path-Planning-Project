/*
 * Map.h
 *
 * Map class
 *
 * Created on 01/12/2019
 * Author: Andre Strobel
 *
 */

#ifndef MAP_H
#define MAP_H

#include <vector>
#include "spline.h"

using std::vector;
using tk::spline;

// map parameters
const unsigned int MAX_TRACK_S = 6945.554;

class Map {

public:
	
	// constructor
	Map() {}
	
	// destructor
	~Map() {}
	
	// initialize map
	void init(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);
	
	// convert xy to Frenet coordinates
	vector<double> xy_to_frenet(const double &x, const double &y, const double &theta);
	
	// convert Frenet to xy coordinates
	vector<double> frenet_to_xy(const double &s, const double &d);

private:
	
	// determine closest waypoint
	vector<unsigned int> closest_waypoint(const double &x, const double &y);
	
	// determine next waypoint
	vector <unsigned int> next_waypoint(const double &x, const double &y, const double &theta);
	
	// waypoint coordinates
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;
	
	// waypoint splines
	spline spline_x_s;
	spline spline_y_s;
	spline spline_dx_s;
	spline spline_dy_s;

};

#endif /* MAP_H_ */