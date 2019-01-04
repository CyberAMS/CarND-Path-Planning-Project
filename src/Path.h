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

class Path {

public:
	
	// constructor
	Path() {}
	
	// destructor
	~Path() {}
	
	// set path
	void set(std::vector<double> path_x, std::vector<double> path_y, double end_path_s, double end_path_d);

private:
	
	// path values
	std::vector<double> path_x;
	std::vector<double> path_y;
	double end_path_s;
	double end_path_d;

};

#endif /* PATH_H_ */