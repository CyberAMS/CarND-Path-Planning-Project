/*
 * helper_functions.h
 *
 * Helper functions
 *
 * Created on 01/05/2019
 * Author: Andre Strobel
 *
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <functional>
#include "Driver.h"

using std::vector;
using std::string;
using std::to_string;
using std::cout;
using std::endl;
using std::ostringstream;

using namespace std::placeholders;

// debugging settings
const bool bDISPLAY = true;
const bool bDISPLAY_DRIVER_PLAN_BEHAVIOR = false;
const bool bDISPLAY_DRIVER_CALCULATE_TRAJECTORY = false;
const bool bDISPLAY_MAP_INIT = true;
const bool bDISPLAY_MAP_CLOSEST_WAYPOINT = true;
const bool bDISPLAY_MAP_NEXT_WAYPOINT = true;
const bool bDISPLAY_MAP_XY_TO_FRENET = true;
const bool bDISPLAY_MAP_FRENET_TO_XY = true;
const bool bDISPLAY_CAR_SET_STATE = false;
const bool bDISPLAY_TRAJECTORY_INIT = true;
const bool bDISPLAY_TRAJECTORY_ADD = true;
const bool bDISPLAY_TRAJECTORY_ADD_FIXED = true;
const bool bDISPLAY_TRAJECTORY_CALCULATE = true;
const bool bDISPLAY_TRAJECTORY_CALCULATE_D_FROM_LANE = false;
const bool bDISPLAY_TRAJECTORY_ESTIMATE_LANES = false;
const bool bDISPLAY_TRAJECTORY_CLOSESTWAYPOINT = true;
const bool bDISPLAY_TRAJECTORY_NEXTWAYPOINT = true;
const bool bDISPLAY_TRAJECTORY_GETFRENET = true;
const bool bDISPLAY_TRAJECTORY_GET_XY = false;
const bool bDISPLAY_PATH_SET = false;
const string DISPLAY_PREFIX = "    ";

// calculate distance
double distance(double x1, double y1, double x2, double y2);

// display double vector as string
string createDoubleVectorString(const vector<double> &double_vector);

// display double vectors as string
string createDoubleVectorsString(const vector<vector<double>> &double_vectors);

// display integer vector as string
string createUnsignedIntegerVectorString(const vector<unsigned int> &int_vector);

// display Cars structure as string
string createCarsString(const Cars &cars);

// display vector of Cars structures as string
string createCarsVectorString(const vector<Cars> &cars_vector);

// sort vector of doubles and return index list
vector<unsigned int> sortDoubleVector(const vector<double> &double_vector);

#endif /* HELPER_FUNCTIONS_H_ */