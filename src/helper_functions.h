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
using std::to_string;
using std::cout;
using std::endl;
using std::ostringstream;

// debugging settings
const bool bDISPLAY = true;

// display double vector as string
string createDoubleVectorString(const vector<double> &double_vector);

// display integer vector as string
string createUnsignedIntegerVectorString(const vector<unsigned int> &int_vector);

#endif /* HELPER_FUNCTIONS_H_ */