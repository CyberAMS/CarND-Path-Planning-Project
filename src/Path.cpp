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
using std::to_string;
using std::cout;
using std::endl;

// set path
void Path::set(vector<double> path_x, vector<double> path_y, double end_path_s, double end_path_d) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_PATH_SET) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "PATH: set - Start" << endl;
		cout << "  path_x, path_y: " << endl << createDoubleVectorsString(vector<vector<double>>{path_x, path_y});
		cout << "  end_path_s: " << end_path_s << endl;
		cout << "  end_path_d: " << end_path_d << endl;
		
	}
	
	Path::path_x = path_x;
	Path::path_y = path_y;
	Path::end_path_s = end_path_s;
	Path::end_path_d = end_path_d;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_PATH_SET) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "--- PATH: set - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// get path_x value
vector<double> Path::get_x() {
	
	return Path::path_x;
	
}

// get path_y value
vector<double> Path::get_y() {
	
	return Path::path_y;
	
}

// display Path object as string
string Path::createString() {
	
	//define variables
	string text = "";
	
	// add information about path to string
	text += DISPLAY_PREFIX + "path_x, path_y =\n" + createDoubleVectorsString(vector<vector<double>>{Path::path_x, Path::path_y});
	text += DISPLAY_PREFIX;
	text += "end_path_s = " + to_string(Path::end_path_s) + " ";
	text += "end_path_d = " + to_string(Path::end_path_d) + "\n";
	
	// return output
	return text;
	
}