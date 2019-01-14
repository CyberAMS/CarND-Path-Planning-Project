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

// convert degrees to radians
double Deg2Rad(const double &x) {
	
	return (x * M_PI / 180);
	
}

// convert radians to degrees
double Rad2Deg(const double &x) {
	
	return (x * 180 / M_PI);
	
}

// convert mph to m_s
double Mph2Ms(const double &speed) {
	
	return (speed * MPH2MS);
	
}

// convert m_s to mph
double Ms2Mph(const double &speed) {
	
	return (speed / MPH2MS);
	
}

// calculate distance
double Distance(const double &x1, const double &y1, const double &x2, const double &y2) {
	
	return sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
	
}

// calculate angle of vector
double GetAngle(const double &x, const double &y) {
	
	return atan2(y, x);
	
}

// calculate magnitude of vector
double GetMagnitude(const double &x, const double &y) {
	
	return sqrt((x * x) + (y * y));
	
}

// calculate x component of vector
double GetX(const double &theta, const double &magnitude) {
	
	return (magnitude * cos(theta));
	
}

// calculate y component of vector
double GetY(const double &theta, const double &magnitude) {
	
	return (magnitude * sin(theta));
	
}

// display double vector as string
string CreateDoubleVectorString(const vector<double> &double_vector) {
	
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

// display double vectors as string
string CreateDoubleVectorsString(const vector<vector<double>> &double_vectors) {
	
	//define variables
	unsigned int current_element = 0;
	unsigned int current_vector = 0;
	ostringstream streamObj;
	string tab = "\t";
	
	// add information about all elements to string
	for (current_element = 0; current_element < double_vectors[0].size(); current_element++) {
		
		streamObj << DISPLAY_PREFIX << "Element " << current_element << ": ";
		
		// add information about all vectors to string
		for (current_vector = 0; current_vector < double_vectors.size(); current_vector++) {
			
			if (current_vector > 0) {
				
				streamObj << tab;
				
			}
			
			streamObj << double_vectors[current_vector][current_element];
			
		}
		
		streamObj << endl;
		
	}
	
	// return output
	return streamObj.str();
	
}

// display integer vector as string
string CreateUnsignedIntegerVectorString(const vector<unsigned int> &int_vector) {
	
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

// sort vector of doubles and return index list
vector<unsigned int> SortDoubleVector(const vector<double> &double_vector) {
	
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