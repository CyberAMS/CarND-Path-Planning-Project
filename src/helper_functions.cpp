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

using std::vector;
using std::string;
using std::to_string;
using std::cout;
using std::endl;
using std::ostringstream;

// display double vector as string
string createDoubleVectorString(const vector<double> &double_vector) {
	
	//define variables
	unsigned int current_element = 0;
	ostringstream streamObj;
	
	// add information about all elements to string
	for (current_element = 0; current_element < double_vector.size(); current_element++) {
		
		streamObj << "Element " << current_element << ": " << double_vector[current_element] << endl;
		
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
		
		text += "Element " + to_string(current_element) + ": " + to_string(int_vector[current_element]) + "\n";
		
	}
	
	// return output
	return text;
	
}