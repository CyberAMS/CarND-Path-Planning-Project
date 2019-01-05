/*
 * Path.cpp
 *
 * Path class
 *
 * Created on 01/03/2019
 * Author: Andre Strobel
 *
 */

#include <iostream>
#include <string>
#include <vector>
#include "helper_functions.h"
#include "Path.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

// set path
void Path::set(vector<double> path_x, vector<double> path_y, double end_path_s, double end_path_d) {
	
	Path::path_x = path_x;
	Path::path_y = path_y;
	Path::end_path_s = end_path_s;
	Path::end_path_d = end_path_d;
	
}