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
#include "Driver.h"

using std::vector;
using std::string;
using std::to_string;
using std::cout;
using std::endl;
using std::ostringstream;

// debugging settings
const bool bDISPLAY = true;
const string DISPLAY_PREFIX = "    ";

// calculate distance
double distance(double x1, double y1, double x2, double y2);

// display double vector as string
string createDoubleVectorString(const vector<double> &double_vector);

// display integer vector as string
string createUnsignedIntegerVectorString(const vector<unsigned int> &int_vector);

// display Cars structure as string
string createCarsString(const Cars &cars);

// display vector of Cars structures as string
string createCarsVectorString(const vector<Cars> &cars_vector);

#endif /* HELPER_FUNCTIONS_H_ */