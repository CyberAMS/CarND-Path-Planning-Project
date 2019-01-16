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
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"

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
using Eigen::MatrixXd;
using Eigen::VectorXd;

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

// determine coefficients for jerk minimizing trajectory
vector<double> JerkMinimizingTrajectoryCoefficients(vector<double> start, vector<double> end, double T) {
	
	// define variables
	double T2 = 0.0;
	double T3 = 0.0;
	double T4 = 0.0;
	double T5 = 0.0;
	MatrixXd T_matrix(3, 3);
	VectorXd sf_diff(3);
	VectorXd coefficients_vector;
	vector<double> coefficients;
	
	// determine time values
	T2 = pow(T, 2);
	T3 = pow(T, 3);
	T4 = pow(T, 4);
	T5 = pow(T, 5);
	
	// determine time matrix
	T_matrix <<     T3,      T4,      T5,
	            3 * T2,  4 * T3,  5 * T4,
	             6 * T, 12 * T2, 20 * T3;
	
	// determine difference based on start and end
	sf_diff << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * pow(T, 2)),
	                                        end[1] - (start[1] + start[2] * T),
	                                                         end[2] - start[2];
	
	// calculate coefficients vector
	coefficients_vector = T_matrix.inverse() * sf_diff;
	
	// determine coefficients
	coefficients = (vector<double>){start[0], start[1], (0.5 * start[2]), coefficients_vector[0], coefficients_vector[1], coefficients_vector[2]};
	
	return coefficients;
	
}

// determine states with jerk minimizing trajectory
vector<double> JerkMinimizingTrajectoryState(vector<double> poly, vector<double> start, double t) {
	
	// define variables
	double t2 = 0.0;
	double t3 = 0.0;
	double t4 = 0.0;
	double t5 = 0.0;
	double state = 0.0;
	double state_d = 0.0;
	double state_dd = 0.0;
	double state_ddd = 0.0;
	
	//initialize outputs
	vector<double> states;
	
	// determine time values
	t2 = pow(t, 2);
	t3 = pow(t, 3);
	t4 = pow(t, 4);
	t5 = pow(t, 5);
		
	// determine states
	state     =      (start[0]) +       (start[1]) * t + (0.5 * start[2]) * t2 +        (poly[3]) * t3 +       (poly[4]) * t4 + (poly[5]) * t5;
	state_d   =      (start[1]) +       (start[2]) * t +  (3.0 * poly[3]) * t2 +  (4.0 * poly[4]) * t3 + (5.0 * poly[5]) * t4;
	state_dd  =      (start[2]) +  (6.0 * poly[3]) * t + (12.0 * poly[4]) * t2 + (20.0 * poly[5]) * t3;
	state_ddd = (6.0 * poly[3]) + (24.0 * poly[4]) * t + (60.0 * poly[5]) * t2;
	
	return (vector<double>){state, state_d, state_dd, state_ddd};
	
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