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

#include <iostream>
#include <string>
#include <vector>
#include "spline.h"

using std::vector;
using std::string;

// map parameters
const double MAX_TRACK_S = 6945.554;

// lane parameters
const double LANE_WIDTH = 4.0;
const double LANE_CENTER_WIDTH = 3.0;
const unsigned int LANE_1 = 1;
const unsigned int LANE_2 = 2;
const unsigned int LANE_3 = 3;
const vector<unsigned int> LANES = {LANE_1, LANE_2, LANE_3};

class Map {

public:
	
	// constructor
	Map() {}
	
	// destructor
	~Map() {}
	
	// initialize map
	void Init(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);
	
	// convert xy to Frenet coordinates
	vector<double> Xy2Frenet(const double &x, const double &y, const double &theta);
	
	// convert Frenet to xy coordinates
	vector<double> Frenet2Xy(const double &s, const double &d);
	vector<vector<double>> Frenet2Xy(const vector<double> &s_values, const vector<double> &d_values);
	
	// assign correct s value considering loop track
	double AssignS(const double &s);
	vector<double> AssignS(const vector<double> &s);
	
	// calculate absolute difference between two s values considering loop track
	double DeltaS(const double &s1, const double &s2);
	vector<double> DeltaS(const vector<double> &s1, const vector<double> &s2);
	
	// display Map object as string
	string CreateString();

private:
	
	// determine closest waypoint
	vector<unsigned int> ClosestWaypoint(const double &x, const double &y);
	
	// determine next waypoint
	vector <unsigned int> NextWaypoint(const double &x, const double &y, const double &theta);
	
	// calculate s value relative to reference considering loop track
	double ReferenceS(const double &s, const double &s_reference);
	vector<double> ReferenceS(const vector<double> &s, const vector<double> &s_reference);
	
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