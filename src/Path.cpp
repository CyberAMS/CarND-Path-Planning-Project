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
void Path::Set(vector<double> path_x, vector<double> path_y, double end_path_s, double end_path_d) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_PATH_SET) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "PATH: Set - Start" << endl;
		cout << "  path_x, path_y: " << endl << CreateDoubleVectorsString(vector<vector<double>>{path_x, path_y});
		cout << "  end_path_s: " << end_path_s << endl;
		cout << "  end_path_d: " << end_path_d << endl;
		
	}
	
	this->path_x = path_x;
	this->path_y = path_y;
	this->end_path_s = end_path_s;
	this->end_path_d = end_path_d;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_PATH_SET) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "--- PATH: Set - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// get path_x value
vector<double> Path::Get_x() {
	
	return this->path_x;
	
}

// get path_y value
vector<double> Path::Get_y() {
	
	return this->path_y;
	
}

// display Path object as string
string Path::CreateString() {
	
	//define variables
	string text = "";
	
	// add information about path to string
	text += DISPLAY_PREFIX + "path_x, path_y =\n" + CreateDoubleVectorsString(vector<vector<double>>{Path::path_x, Path::path_y});
	text += DISPLAY_PREFIX;
	text += "end_path_s = " + to_string(Path::end_path_s) + " ";
	text += "end_path_d = " + to_string(Path::end_path_d) + "\n";
	
	// return output
	return text;
	
}