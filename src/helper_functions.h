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

using std::vector;
using std::string;

// debug settings
const bool bFILEOUTPUT = true;
const string OUTPUT_FILENAME = "out.txt";
const bool bDISPLAY = true;
const bool bDISPLAY_DRIVER_PLANBEHAVIOR = true;
const bool bDISPLAY_DRIVER_SETVEHICLES = true;
const bool bDISPLAY_MAP_INIT = false;
const bool bDISPLAY_MAP_XY2FRENET = true;
const bool bDISPLAY_MAP_FRENET2XY = true;
const bool bDISPLAY_MAP_CLOSESTWAYPOINT = true;
const bool bDISPLAY_MAP_NEXTWAYPOINT = true;
const bool bDISPLAY_VEHICLE_UPDATE = true;
const bool bDISPLAY_VEHICLE_AHEAD = true;
const bool bDISPLAY_VEHICLE_BEHIND = true;
const bool bDISPLAY_VEHICLE_GETLANED = true;
const bool bDISPLAY_VEHICLE_DETERMINELANE = true;
const bool bDISPLAY_VEHICLE_CHECKINSIDELANE = true;
const bool bDISPLAY_PATH_SET = true;
const bool bDISPLAY_TRAJECTORY_INIT = true;
const bool bDISPLAY_TRAJECTORY_ADD = true;
const bool bDISPLAY_TRAJECTORY_ADDJERKMINIMIZINGTRAJECTORY = true;
const bool bDISPLAY_TRAJECTORY_GENERATE = true;
const bool bDISPLAY_TRAJECTORY_VALID = true;
const bool bDISPLAY_TRAJECTORY_COST = true;
const bool bDISPLAY_TRAJECTORY_REMOVEFIRSTSTEPS = true;
const bool bDISPLAY_STATE_INIT = true;
const bool bDISPLAY_STATE_SETBEHAVIOR = true;
const bool bDISPLAY_STATE_GETNEXTPOSSIBLEBEHAVIORS = true;
const bool bDISPLAY_STATE_GENERATETRAJECTORYFROMBEHAVIOR = true;
const string DISPLAY_PREFIX = "    ";

// define constants
const double MPH2MS = 0.44704; // factor between miles per hour and meters per second

// convert degrees to radians
double Deg2Rad(const double &x);

// convert radians to degrees
double Rad2Deg(const double &x);

// convert mph to m_s
double Mph2Ms(const double &speed);

// convert m_s to mph
double Ms2Mph(const double &speed);

// calculate distance
double Distance(const double &x1, const double &y1, const double &x2, const double &y2);

// calculate angle of vector
double GetAngle(const double &x, const double &y);

// calculate angle of vector
double GetMagnitude(const double &x, const double &y);

// calculate angle of vector
double GetX(const double &theta, const double &magnitude);

// calculate angle of vector
double GetY(const double &theta, const double &magnitude);

// determine coefficients for jerk minimizing trajectory
vector<double> JerkMinimizingTrajectoryCoefficients(vector<double> start, vector<double> end, double T);

// determine states with jerk minimizing trajectory
vector<double> JerkMinimizingTrajectoryState(vector<double> poly, vector<double> start, double t);

// display double vector as string
string CreateDoubleVectorString(const vector<double> &double_vector);

// display double vectors as string
string CreateDoubleVectorsString(const vector<vector<double>> &double_vectors);

// display integer vector as string
string CreateUnsignedIntegerVectorString(const vector<unsigned int> &int_vector);

// sort vector of doubles and return index list
vector<unsigned int> SortDoubleVector(const vector<double> &double_vector);

#endif /* HELPER_FUNCTIONS_H_ */