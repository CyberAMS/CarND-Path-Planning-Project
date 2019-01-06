/*
 * Path.h
 *
 * Path class
 *
 * Created on 01/03/2019
 * Author: Andre Strobel
 *
 */

#ifndef PATH_H_
#define PATH_H_

#include <iostream>
#include <string>
#include <vector>
#include "helper_functions.h"

using std::vector;
using std::string;
using std::to_string;
using std::cout;
using std::endl;

class Path {

public:
	
	// constructor
	Path() {}
	
	// destructor
	~Path() {}
	
	// set path
	void set(vector<double> path_x, vector<double> path_y, double end_path_s, double end_path_d);
	
	// get path_x value
	vector<double> get_x();
	
	// get path_y value
	vector<double> get_y();
	
	// display Path object as string
	string createString();

private:
	
	// path values
	vector<double> path_x;
	vector<double> path_y;
	double end_path_s;
	double end_path_d;

};

#endif /* PATH_H_ */