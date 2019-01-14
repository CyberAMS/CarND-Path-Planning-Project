/*
 * Vehicle.cpp
 *
 * Vehicle class
 *
 * Created on 01/13/2019
 * Author: Andre Strobel
 *
 */

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include "Vehicle.h"
#include "Map.h"
#include "helper_functions.h"

using std::vector;
using std::string;
using std::to_string;
using std::cout;
using std::endl;
using std::ostringstream;
using std::fabs;

// constructor
Vehicle::Vehicle() {}
Vehicle::Vehicle(unsigned int id, double x, double y, double vx, double vy, double s, double d) {
	
	this->Update(id, x, y, vx, vy, s, d);
	
}
Vehicle::Vehicle(double x, double y, double s, double d, double theta, double v) {
	
	this->Update(x, y, s, d, theta, v);
	
}

// set state of car
void Vehicle::Update(unsigned int id, double x, double y, double vx, double vy, double s, double d) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_UPDATE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: Update - Start" << endl;
		cout << "  id: " << id << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << y << endl;
		cout << "  vx: " << vx << endl;
		cout << "  vy: " << vy << endl;
		cout << "  s: " << s << endl;
		cout << "  d: " << d << endl;
		
	}
	
	// set state with inputs
	this->id = id;
	this->x = x;
	this->y = y;
	this->vx = vx;
	this->vy = vy;
	this->s = s;
	this->d = d;
	
	// calculate missing states
	this->theta = GetAngle(vx, vy);
	this->v = GetMagnitude(vx, vy);
	
	// determine values
	this->Update();
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_UPDATE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- VEHICLE: Update - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}
void Vehicle::Update(double x, double y, double s, double d, double theta, double v) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_UPDATE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: Update - Start" << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << y << endl;
		cout << "  vx: " << vx << endl;
		cout << "  vy: " << vy << endl;
		cout << "  theta: " << theta << endl;
		cout << "  v: " << v << endl;
		
	}
	
	// set state with inputs
	this->id = EGO_CAR_ID;
	this->x = x;
	this->y = y;
	this->s = s;
	this->d = d;
	this->theta = theta;
	this->v = v;
	
	// calculate missing states
	this->vx = GetX(theta, v);
	this->vy = GetY(theta, v);
	
	// determine values
	this->Update();
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_UPDATE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- VEHICLE: Update - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}
void Vehicle::Update() {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_UPDATE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: Update - Start" << endl;
		
	}
	
	// determine values
	this->is_inside_lane = this->CheckInsideLane();
	this->lane = this->DetermineLane();
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_UPDATE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- VEHICLE: Update - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// determine lane
unsigned int Vehicle::DetermineLane() {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_DETERMINELANE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: DetermineLane - Start" << endl;
		
	}
	
	// initialize outputs
	unsigned int lane = this->DetermineLane(this->d);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_DETERMINELANE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  lane: " << lane << endl;
		cout << "--- VEHICLE: DetermineLane - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return lane;
	
}
unsigned int Vehicle::DetermineLane(const double &d) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_DETERMINELANE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: DetermineLane - Start" << endl;
		cout << "  d: " << d << endl;
		
	}
	
	// define variables
	vector<double> lane_centers;
	double min_distance = std::numeric_limits<double>::max();
	unsigned int lane = 0;
	unsigned int count = 0;
	double lane_distance = 0;
	
	// get center for all lanes
	lane_centers = this->GetLaneD(LANES);
	
	// check all lanes for minimum distance to current distance value
	for (count = 0; count < lane_centers.size(); count++) {
		
		lane_distance = fabs(d - lane_centers[count]);
		
		if (lane_distance < min_distance) {
			
			min_distance = lane_distance;
			lane = LANES[count];
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_DETERMINELANE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  lane: " << lane << endl;
		cout << "--- VEHICLE: DetermineLane - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return lane;
	
}

// get vehicles ahead of own vehicle
vector<Vehicle> Vehicle::Ahead(const vector<Vehicle> &vehicles, unsigned int lane) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_AHEAD) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: Ahead - Start" << endl;
		cout << "  vehicles: " << endl << this->CreateVehiclesVectorString(vehicles);
		cout << "  lane: " << lane << endl;
		
	}
	
	// initialize outputs
	vector<Vehicle> vehicles_ahead;
	
	// check all vehicles
	for (const Vehicle &vehicle : vehicles) {
		
		// vehicle is in given lane and in front of own vehicle
		if ((vehicle.lane == lane) && (vehicle.s > this->s)) {
			
			vehicles_ahead.push_back(vehicle);
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_AHEAD) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  vehicles_ahead: " << endl << this->CreateVehiclesVectorString(vehicles_ahead);
		cout << "--- VEHICLE: Ahead - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return vehicles_ahead;
}

// get vehicles behind own vehicle
vector<Vehicle> Vehicle::Behind(const vector<Vehicle> &vehicles, unsigned int lane) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_BEHIND) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: Behind - Start" << endl;
		cout << "  vehicles: " << endl << this->CreateVehiclesVectorString(vehicles);
		cout << "  lane: " << lane << endl;
		
	}
	
	// initialize outputs
	vector<Vehicle> vehicles_behind;
	
	// check all vehicles
	for (const Vehicle &vehicle : vehicles) {
		
		// vehicle is in given lane and behind own vehicle
		if ((vehicle.lane == lane) && (vehicle.s < this->s)) {
			
			vehicles_behind.push_back(vehicle);
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_BEHIND) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  vehicles_behind: " << endl << this->CreateVehiclesVectorString(vehicles_behind);
		cout << "--- VEHICLE: Behind - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return vehicles_behind;
}

// get x value
double Vehicle::Get_x() {
	
	return this->x;
	
}

// get y value
double Vehicle::Get_y() {
	
	return this->y;
	
}

// get vx value
double Vehicle::Get_vx() {
	
	return this->vx;
	
}

// get vy value
double Vehicle::Get_vy() {
	
	return this->vy;
	
}

// get s value
double Vehicle::Get_s() {
	
	return this->s;
	
}

// get d value
double Vehicle::Get_d() {
	
	return this->d;
	
}

// get yaw angle
double Vehicle::Get_theta() {
	
	return this->theta;
	
}

// get speed
double Vehicle::Get_v() {
	
	return this->v;
	
}

// get lane value
unsigned int Vehicle::Get_lane() {
	
	return this->lane;
	
}

// get is_inside_lane value
bool Vehicle::Get_is_inside_lane() {
	
	return this->is_inside_lane;
	
}

// display Vehicle object as string
string Vehicle::CreateString() {
	
	//define variables
	string text = "";
	
	// add information about car to string
	text += DISPLAY_PREFIX;
	text += "id = " + to_string(this->id) + " ";
	text += "x = " + to_string(this->x) + " ";
	text += "y = " + to_string(this->y) + " ";
	text += "vx = " + to_string(this->vx) + " ";
	text += "vy = " + to_string(this->vy) + " ";
	text += "s = " + to_string(this->s) + " ";
	text += "d = " + to_string(this->d) + " ";
	text += "theta = " + to_string(this->theta) + " ";
	text += "v = " + to_string(this->v) + " ";
	text += "lane = " + to_string(this->lane) + " ";
	text += "is_inside_lane = " + to_string(this->is_inside_lane) + "\n";
	
	// return output
	return text;
	
}

// display vector of Vehicle objects as string
string Vehicle::CreateVehiclesVectorString(vector<Vehicle> vehicles_vector) {
	
	//define variables
	unsigned int current_element = 0;
	string text = "";
	
	// add information about all cars to string
	for (current_element = 0; current_element < vehicles_vector.size(); current_element++) {
		
		text += DISPLAY_PREFIX + "Element " + to_string(current_element) + ": " + vehicles_vector[current_element].CreateString();
		
	}
	
	// return output
	return text;
	
}

// get d values for lanes
vector<double> Vehicle::GetLaneD(const vector<unsigned int> &lanes) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_GETLANED) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: GetLaneD - Start" << endl;
		cout << "  lanes: " << endl << CreateUnsignedIntegerVectorString(lanes);
		
	}
	
	// define variables
	unsigned int count = 0;
	double d = 0;
	
	// initialize outputs
	vector<double> d_values;
	
	for (count = 0; count < lanes.size(); count++) {
		
		d = (LANE_WIDTH / 2) + (LANE_WIDTH * ((double) lanes[count] - 1));
		d_values.push_back(d);
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_GETLANED) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  d_values: " << endl << CreateDoubleVectorString(d_values);
		cout << "--- VEHICLE: GetLaneD - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return d_values;
	
}

// check whether vehicle is inside the determined lane
bool Vehicle::CheckInsideLane() {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_CHECKINSIDELANE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: CheckInsideLane - Start" << endl;
		
	}
	
	// define variables
	vector<double> d_values;
	double lane_center;
	
	// initialize outputs
	bool is_inside_lane = false;
	
	// get lane center d value of vehicle's lane
	d_values = GetLaneD((vector<unsigned int>){this->lane});
	lane_center = d_values[0];
	
	// check whether vehicle is within lane center range
	if (fabs(this->d - lane_center) <= (LANE_CENTER_WIDTH / 2)) {
		
		is_inside_lane = true;
		
	} else {
		
		is_inside_lane = false;
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_CHECKINSIDELANE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  is_inside_lane: " << is_inside_lane << endl;
		cout << "--- VEHICLE: CheckInsideLane - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return is_inside_lane;
	
}