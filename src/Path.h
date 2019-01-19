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

using std::vector;
using std::string;

class Path {

public:
	
	// constructor
	Path() {}
	
	// destructor
	~Path() {}
	
	// set path
	void Set(vector<double> path_x, vector<double> path_y, double end_path_s, double end_path_d);
	
	// get path_x value
	vector<double> Get_x();
	vector<double>* Get_x_ptr();
	
	// get path_y value
	vector<double> Get_y();
	vector<double>* Get_y_ptr();
	
	// get end_path_s value
	double Get_end_path_s();
	double* Get_end_path_s_ptr();
	
	// get end_path_d value
	double Get_end_path_d();
	double* Get_end_path_d_ptr();
	
	// display Path object as string
	string CreateString();

private:
	
	// path values
	vector<double> path_x;
	vector<double> path_y;
	double end_path_s = 0;
	double end_path_d = 0;

};

#endif /* PATH_H_ */