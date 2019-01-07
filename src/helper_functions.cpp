/*
 * helper_functions.cpp
 *
 * Helper functions
 *
 * Created on 01/05/2019
 * Author: Andre Strobel
 *
 */

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <functional>
#include "helper_functions.h"
#include "Driver.h"

using std::vector;
using std::string;
using std::to_string;
using std::cout;
using std::endl;
using std::ostringstream;
using std::sqrt;
using std::iota;
using std::sort;
using std::begin;
using std::end;
using std::copy;

using namespace std::placeholders;

// calculate distance
double distance(double x1, double y1, double x2, double y2) {
	
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	
}

// display double vector as string
string createDoubleVectorString(const vector<double> &double_vector) {
	
	//define variables
	unsigned int current_element = 0;
	ostringstream streamObj;
	
	// add information about all elements to string
	for (current_element = 0; current_element < double_vector.size(); current_element++) {
		
		streamObj << DISPLAY_PREFIX << "Element " << current_element << ": " << double_vector[current_element] << endl;
		
	}
	
	// return output
	return streamObj.str();
	
}

// display integer vector as string
string createUnsignedIntegerVectorString(const vector<unsigned int> &int_vector) {
	
	//define variables
	unsigned int current_element = 0;
	string text = "";
	
	// add information about all elements to string
	for (current_element = 0; current_element < int_vector.size(); current_element++) {
		
		text += DISPLAY_PREFIX + "Element " + to_string(current_element) + ": " + to_string(int_vector[current_element]) + "\n";
		
	}
	
	// return output
	return text;
	
}

// display Cars structure as string
string createCarsString(const Cars &cars) {
	
	//define variables
	string text = "";
	
	// add information about cars to string
	text += DISPLAY_PREFIX;
	text += "id=" + to_string(cars.id) + " ";
	text += "x=" + to_string(cars.x) + " ";
	text += "y=" + to_string(cars.y) + " ";
	text += "vx=" + to_string(cars.vx) + " ";
	text += "vy=" + to_string(cars.vy) + " ";
	text += "s=" + to_string(cars.s) + " ";
	text += "d=" + to_string(cars.d) + "\n";
	
	// return output
	return text;
	
}

// display vector of Cars structures as string
string createCarsVectorString(const vector<Cars> &cars_vector) {
	
	//define variables
	unsigned int current_element = 0;
	string text = "";
	
	// add information about all cars to string
	for (current_element = 0; current_element < cars_vector.size(); current_element++) {
		
		text += DISPLAY_PREFIX + "Element " + to_string(current_element) + ": " + createCarsString(cars_vector[current_element]);
		
	}
	
	// return output
	return text;
	
}

// sort vector of doubles and return index list
vector<unsigned int> sortDoubleVector(const vector<double> &double_vector) {
	
	// define variables
	double data[double_vector.size()];
	vector<unsigned int> index (double_vector.size());
	
	// copy data to array
	copy(double_vector.begin(), double_vector.end(), data);
	
	// fill index with increasing numbers starting with 0
	iota(begin(index), end(index), 0);
	
	// sort array and retrieve sorted indices
	sort(begin(index), end(index), [&](int i1, int i2) {return data[i1] < data[i2];});
	
	return index;
	
}