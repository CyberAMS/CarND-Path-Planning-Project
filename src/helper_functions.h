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
const bool bDISPLAY_DRIVER_SETVEHICLES = false;
const bool bDISPLAY_MAP_INIT = false;
const bool bDISPLAY_MAP_XY2FRENET = false;
const bool bDISPLAY_MAP_FRENET2XY = false;
const bool bDISPLAY_MAP_CLOSESTWAYPOINT = false;
const bool bDISPLAY_MAP_NEXTWAYPOINT = false;
const bool bDISPLAY_VEHICLE_UPDATE = false;
const bool bDISPLAY_VEHICLE_AHEAD = false;
const bool bDISPLAY_VEHICLE_BEHIND = false;
const bool bDISPLAY_VEHICLE_GETLANED = false;
const bool bDISPLAY_VEHICLE_DETERMINELANE = false;
const bool bDISPLAY_VEHICLE_CHECKINSIDELANE = false;
const bool bDISPLAY_PATH_SET = false;
const bool bDISPLAY_TRAJECTORY_INIT = true;
const bool bDISPLAY_TRAJECTORY_START = false;
const bool bDISPLAY_TRAJECTORY_ADD = false;
const bool bDISPLAY_TRAJECTORY_ADDJERKMINIMIZINGTRAJECTORY = false;
const bool bDISPLAY_TRAJECTORY_GENERATE = false;
const bool bDISPLAY_TRAJECTORY_VALID = true;
const bool bDISPLAY_TRAJECTORY_COST = false;
const bool bDISPLAY_TRAJECTORY_REMOVEFIRSTSTEPS = false;
const bool bDISPLAY_TRAJECTORY_KEEPFIRSTSTEPS = false;
const bool bDISPLAY_STATE_INIT = true;
const bool bDISPLAY_STATE_SETBEHAVIOR = false;
const bool bDISPLAY_STATE_GETNEXTPOSSIBLEBEHAVIORS = true;
const bool bDISPLAY_STATE_GENERATETRAJECTORYFROMBEHAVIOR = true;
const string DISPLAY_PREFIX = "    ";
const unsigned int DISPLAY_COLUMN_WIDTH = 15;

// define constants
const double MPH2MS = 0.44704; // factor between miles per hour and meters per second
const double ZERO_DIFFERENTIAL_VALUE = 0.0;
const double NEUTRAL_GAIN = 1.0;

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
double Angle(const double &x, const double &y);
vector<double> Angle(const vector<double> &x_vector, const vector<double> &y_vector);

// calculate angle of vector
double Magnitude(const double &x, const double &y);
vector<double> Magnitude(const vector<double> &x_vector, const vector<double> &y_vector);

// calculate angle of vector
double GetX(const double &theta, const double &magnitude);

// calculate angle of vector
double GetY(const double &theta, const double &magnitude);

// calculate average of double vector
double AbsAverage(const vector<double> &vector_data);

// calculate maximum of double vector
double Maximum(const vector<double> &vector_data);

// calculate minimum of double vector
double Minimum(const vector<double> &vector_data);

// calculate sum of double vector
double Sum(const vector<double> &vector_data);

// calculate differential of double vector
vector<double> Differential(const vector<double> &vector_data);

// add number to double vector
vector<double> Addition(const vector<double> &vector_data, const double &add);

// multiply double vector with gain
vector<double> Multiply(const vector<double> &vector_data, const double &gain);

// accumulate double vector
vector<double> Accumulate(const vector<double> &vector_data);

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