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
#include "Path.h"

using namespace std;

// set path
void Path::set(std::vector<double> path_x, std::vector<double> path_y, double end_path_s, double end_path_d) {
	
	Path::path_x = path_x;
	Path::path_y = path_y;
	Path::end_path_s = end_path_s;
	Path::end_path_d = end_path_d;
	
}