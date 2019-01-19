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
#include "Trajectory.h"
#include "helper_functions.h"

using std::vector;
using std::string;
using std::to_string;
using std::cout;
using std::endl;
using std::ostringstream;
using std::fabs;
using std::min;

// constructor
Vehicle::Vehicle() {}
Vehicle::Vehicle(Map map, unsigned int id, double x, double y, double vx, double vy, double s, double d) {
	
	this->Update(map, id, x, y, vx, vy, s, d);
	
}
Vehicle::Vehicle(double x, double y, double s, double d, double theta, double v) {
	
	this->Update(x, y, s, d, theta, v);
	
}

// set state of car
void Vehicle::Update(Map map, unsigned int id, double x, double y, double vx, double vy, double s, double d) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_UPDATE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: Update - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		cout << "  id: " << id << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << y << endl;
		cout << "  vx: " << vx << endl;
		cout << "  vy: " << vy << endl;
		cout << "  s: " << s << endl;
		cout << "  d: " << d << endl;
		
	}
	
	// define variables
	vector<double> svdv_start;
	vector<double> svdv_end;
	double s_start = 0.0;
	double sv_start = 0.0;
	double d_start = 0.0;
	double dv_start = 0.0;
	double theta_start = 0.0;
	
	// set state with inputs
	this->id = id;
	this->x = x;
	this->y = y;
	this->vx = vx;
	this->vy = vy;
	this->s = s;
	this->d = d;
	
	// calculate missing states
	this->theta = Angle(vx, vy);
	this->v = Magnitude(vx, vy);
	
	// get Frenet position
	s_start = this->Get_s();
	d_start = this->Get_d();
	theta_start = this->Get_theta();
	
	// determine Frenet velocities
	svdv_start = map.Xy2Frenet(this->Get_x(), this->Get_y(), theta_start);
	svdv_end = map.Xy2Frenet((this->Get_x() + this->Get_vx()), (this->Get_y() + this->Get_vy()), theta_start);
	sv_start = svdv_end[0] - svdv_start[0];
	dv_start = svdv_end[1] - svdv_start[1];
	
	// predict future trajectory
	this->PredictTrajectory(map, s_start, sv_start, d_start, dv_start, theta_start);
	
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
		cout << "  s: " << s << endl;
		cout << "  d: " << d << endl;
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
	this->lane = this->DetermineLane();
	this->is_inside_lane = this->CheckInsideLane();
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_UPDATE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- VEHICLE: Update - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// predict future trajectory
void Vehicle::PredictTrajectory(Map map, const double &s_start, const double &sv_start, const double &d_start, const double &dv_start, const double &theta_start) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_PREDICTTRAJECTORY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: PredictTrajectory - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		cout << "  s_start: " << s_start << endl;
		cout << "  sv_start: " << sv_start << endl;
		cout << "  d_start: " << d_start << endl;
		cout << "  dv_start: " << dv_start << endl;
		cout << "  theta_start: " << theta_start << endl;
		
	}
	
	// define variables
	double s_next = 0.0;
	double sv_next = 0.0;
	double sa_next = 0.0;
	double sj_next = 0.0;
	double d_next = 0.0;
	vector<double> xy_next;
	double x_next = 0.0;
	double y_next = 0.0;
	double da_start = 0.0;
	double dj_start = 0.0; 
	double s_target = 0.0;
	double sv_target = 0.0;
	double sa_target = 0.0;
	double d_target = 0.0;
	double dv_target = 0.0;
	double da_target = 0.0;
	
	// initialize outputs
	Trajectory predicted_trajectory;
	
	// add current vehicle position at next step to trajectory
	s_next = s_start + (sv_start * SAMPLE_TIME);
	sv_next = sv_start;
	sa_next = (sv_next / SAMPLE_TIME);
	sj_next = (sa_next / SAMPLE_TIME);
	d_next = d_start;
	xy_next = map.Frenet2Xy(s_next, d_next);
	x_next = xy_next[0];
	y_next = xy_next[1];
	predicted_trajectory.Start(x_next, y_next, s_next, sv_next, sa_next, sj_next, d_next, dv_start, da_start, dj_start, theta_start);
	
	// determine target values
	sv_target = sv_start;
	//s_target = s_start + (Average((vector<double>){sv_target, sv_start}) * STEP_TIME_INTERVAL);
	s_target = s_start + (sv_target * STEP_TIME_INTERVAL);
	dv_target = dv_start;
	//d_target = d_start + (Average((vector<double>){dv_target, dv_start}) * STEP_TIME_INTERVAL);
	d_target = d_start + (dv_target * STEP_TIME_INTERVAL);
	
	// add jerk minimizing trajectory
	predicted_trajectory.AddJerkMinimizingTrajectory(map, s_target, sv_target, sa_target, d_target, dv_target, da_target);
	
	// prevent to reinitialize this trajectory
	predicted_trajectory.Set_is_initialized(true);
	
	// use predicted trajectory for current vehicle
	this->SetTrajectory(predicted_trajectory);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_PREDICTTRAJECTORY) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  s_target: " << s_target << endl;
		cout << "  sv_target: " << sv_target << endl;
		cout << "  d_target: " << d_target << endl;
		cout << "  dv_target: " << dv_target << endl;
		cout << "  this->trajectory: " << endl << this->Get_trajectory().CreateString();
		cout << "--- VEHICLE: PredictTrajectory - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// get d values for lanes
double Vehicle::GetLaneD(const unsigned int &lane) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_GETLANED) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: GetLaneD - Start" << endl;
		cout << "  lane: " << lane << endl;
		
	}
	
	// initialize outputs
	double d = 0;
	
	d = (LANE_WIDTH / 2) + (LANE_WIDTH * ((double)lane - 1));
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_GETLANED) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  d: " << d << endl;
		cout << "--- VEHICLE: GetLaneD - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return d;
	
}
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
		
		d = this->GetLaneD(lanes[count]);
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

// determine lane
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
		
		// smallest distance and distance within lane width around lane center
		if ((lane_distance < min_distance) && (lane_distance <= (LANE_WIDTH / 2))){
			
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

// check whether vehicle is inside the determined lane
bool Vehicle::CheckInsideLane(const double &d, const unsigned int &lane) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_CHECKINSIDELANE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: CheckInsideLane - Start" << endl;
		cout << "  d: " << d << endl;
		cout << "  lane: " << lane << endl;
		
	}
	
	// define variables
	double lane_center;
	
	// initialize outputs
	bool is_inside_lane = false;
	
	// get lane center d value of vehicle's lane
	lane_center = GetLaneD(lane);
	
	// check whether vehicle is within lane center range
	if (fabs(d - lane_center) <= (LANE_CENTER_WIDTH / 2)) {
		
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
bool Vehicle::CheckInsideLane() {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_CHECKINSIDELANE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: CheckInsideLane - Start" << endl;
		
	}
	
	// initialize outputs
	bool is_inside_lane = false;
	
	// check this vehicle for being inside its lane
	is_inside_lane = this->CheckInsideLane(this->d, this->lane);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_CHECKINSIDELANE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  is_inside_lane: " << is_inside_lane << endl;
		cout << "--- VEHICLE: CheckInsideLane - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return is_inside_lane;
	
}

// get vehicles ahead of own vehicle
vector<Vehicle> Vehicle::Ahead(const vector<Vehicle> &vehicles, const unsigned int &lane) {
	
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
vector<Vehicle> Vehicle::Behind(const vector<Vehicle> &vehicles, const unsigned int &lane) {
	
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

// detect collision between trajectory of own vehicle and trajectories of other vehicles
bool Vehicle::DetectCollision(vector<Vehicle> vehicles) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_DETECTCOLLISION) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: DetectCollision - Start" << endl;
		cout << "  vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(vehicles);
		
	}
	
	// define variables
	unsigned int count_t = 0;
	double ego_front_left_s = 0.0;
	double ego_front_left_d = 0.0;
	double ego_front_right_s = 0.0;
	double ego_front_right_d = 0.0;
	double ego_rear_right_s = 0.0;
	double ego_rear_right_d = 0.0;
	double ego_rear_left_s = 0.0;
	double ego_rear_left_d = 0.0;
	unsigned int count_v = 0;
	double vehicle_front_left_s = 0.0;
	double vehicle_front_left_d = 0.0;
	double vehicle_front_right_s = 0.0;
	double vehicle_front_right_d = 0.0;
	double vehicle_rear_right_s = 0.0;
	double vehicle_rear_right_d = 0.0;
	double vehicle_rear_left_s = 0.0;
	double vehicle_rear_left_d = 0.0;
	
	// initialize outputs
	bool collision = false;
	
	// investigate all trajectory steps
	for (count_t = 0; count_t < min(this->Get_trajectory().Get_s().size(), trajectory.Get_s().size()); count_t++) {
		
		// get corners of safety box around own vehicle
		ego_front_left_s = this->Get_trajectory().Get_s()[count_t] + SAFETY_BOX_DISTANCE;
		ego_front_left_d = this->Get_trajectory().Get_d()[count_t] - (STANDARD_VEHICLE_WIDTH / 2) - SAFETY_BOX_DISTANCE;
		ego_front_right_s = ego_front_left_s;
		ego_front_right_d = this->Get_trajectory().Get_d()[count_t] + (STANDARD_VEHICLE_WIDTH / 2) + SAFETY_BOX_DISTANCE;
		ego_rear_right_s = this->Get_trajectory().Get_s()[count_t] - STANDARD_VEHICLE_LENGTH - SAFETY_BOX_DISTANCE;
		ego_rear_right_d = ego_front_right_d;
		ego_rear_left_s = ego_rear_right_s;
		ego_rear_left_d = ego_front_left_d;
		
		// check all vehicles
		for (count_v = 0; count_v < vehicles.size(); count_v++) {
			
			// get corners of current vehicle
			vehicle_front_left_s = vehicles[count_v].Get_trajectory().Get_s()[count_t];
			vehicle_front_left_d = vehicles[count_v].Get_trajectory().Get_d()[count_t] - (STANDARD_VEHICLE_WIDTH / 2);
			vehicle_front_right_s = vehicle_front_left_s;
			vehicle_front_right_d = vehicles[count_v].Get_trajectory().Get_d()[count_t] + (STANDARD_VEHICLE_WIDTH / 2);
			vehicle_rear_right_s = vehicles[count_v].Get_trajectory().Get_s()[count_t] - STANDARD_VEHICLE_LENGTH;
			vehicle_rear_right_d = vehicle_front_right_d;
			vehicle_rear_left_s = vehicle_rear_right_s;
			vehicle_rear_left_d = vehicle_front_left_d;
			
			// check whether both rectangles overlap
			if (((ego_front_left_s >= vehicle_rear_right_s) && (vehicle_front_left_s >= ego_rear_right_s)) && ((ego_front_left_d >= vehicle_rear_right_d) && (vehicle_front_left_d >= ego_rear_right_d))) {
				
				collision = true;
				
			}
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_DETECTCOLLISION) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  collision: " << collision << endl;
		cout << "--- VEHICLE: DetectCollision - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return collision;
	
}

// determine collision cost
double Vehicle::CostCollision(vector<Vehicle> vehicles, const double &weight) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_COSTCOLLISION) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: CostCollision - Start" << endl;
		cout << "  vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(vehicles);
		cout << "  weight: " << weight << endl;
		
	}
	
	// define variables
	double cost = std::numeric_limits<double>::max();
	
	// check for collision and adjust cost
	if (this->DetectCollision(vehicles)) {
		
		// add cost
		cost = weight;
		
	} else {
		
		// no cost
		cost = ZERO_COST;
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_COSTCOLLISION) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  cost: " << cost << endl;
		cout << "--- VEHICLE: CostCollision - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return cost;
	
}

// // determine cost of trajectory for own vehicle
	double Vehicle::TrajectoryCost(Trajectory trajectory, vector<Vehicle> vehicles) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_TRAJECTORYCOST) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: TrajectoryCost - Start" << endl;
		cout << "  trajectory: " << endl << trajectory.CreateString();
		cout << "  vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(vehicles);
		
	}
	
	// define variables
	double cost = std::numeric_limits<double>::max();
	
	// add collision costs
	cost =+ this->CostCollision(vehicles, COST_COLLISON_WEIGHT);
	
	
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_TRAJECTORYCOST) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  cost: " << cost << endl;
		cout << "--- VEHICLE: TrajectoryCost - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return cost;
	
}

// set trajectory
void Vehicle::SetTrajectory(Trajectory trajectory) {
	
	this->trajectory = trajectory;
	
}

// get id number
unsigned int Vehicle::Get_id() {
	
	return this->id;
	
}
unsigned int* Vehicle::Get_id_ptr() {
	
	return &this->id;
	
}

// get x value
double Vehicle::Get_x() {
	
	return this->x;
	
}
double* Vehicle::Get_x_ptr() {
	
	return &this->x;
	
}

// get y value
double Vehicle::Get_y() {
	
	return this->y;
	
}
double* Vehicle::Get_y_ptr() {
	
	return &this->y;
	
}

// get vx value
double Vehicle::Get_vx() {
	
	return this->vx;
	
}
double* Vehicle::Get_vx_ptr() {
	
	return &this->vx;
	
}

// get vy value
double Vehicle::Get_vy() {
	
	return this->vy;
	
}
double* Vehicle::Get_vy_ptr() {
	
	return &this->vy;
	
}

// get s value
double Vehicle::Get_s() {
	
	return this->s;
	
}
double* Vehicle::Get_s_ptr() {
	
	return &this->s;
	
}

// get d value
double Vehicle::Get_d() {
	
	return this->d;
	
}
double* Vehicle::Get_d_ptr() {
	
	return &this->d;
	
}

// get yaw angle
double Vehicle::Get_theta() {
	
	return this->theta;
	
}
double* Vehicle::Get_theta_ptr() {
	
	return &this->theta;
	
}

// get speed
double Vehicle::Get_v() {
	
	return this->v;
	
}
double* Vehicle::Get_v_ptr() {
	
	return &this->v;
	
}

// get lane value
unsigned int Vehicle::Get_lane() {
	
	return this->lane;
	
}
unsigned int* Vehicle::Get_lane_ptr() {
	
	return &this->lane;
	
}

// get is_inside_lane value
bool Vehicle::Get_is_inside_lane() {
	
	return this->is_inside_lane;
	
}
bool* Vehicle::Get_is_inside_lane_ptr() {
	
	return &this->is_inside_lane;
	
}

// get trajectory
Trajectory Vehicle::Get_trajectory() {
	
	return this->trajectory;
	
}
Trajectory* Vehicle::Get_trajectory_ptr() {
	
	return &this->trajectory;
	
}

// get width
double Vehicle::Get_width() {
	
	return this->width;
	
}
double* Vehicle::Get_width_ptr() {
	
	return &this->width;
	
}

// get length
double Vehicle::Get_length() {
	
	return this->length;
	
}
double* Vehicle::Get_length_ptr() {
	
	return &this->length;
	
}

// display Vehicle object as string
string Vehicle::CreateString() {
	
	//define variables
	string text = "";
	
	// add information about car to string
	text += DISPLAY_PREFIX;
	text += "id = " + to_string(this->Get_id()) + " ";
	text += "x = " + to_string(this->Get_x()) + " ";
	text += "y = " + to_string(this->Get_y()) + " ";
	text += "vx = " + to_string(this->Get_vx()) + " ";
	text += "vy = " + to_string(this->Get_vy()) + " ";
	text += "s = " + to_string(this->Get_s()) + " ";
	text += "d = " + to_string(this->Get_d()) + " ";
	text += "theta = " + to_string(this->Get_theta()) + " ";
	text += "v = " + to_string(this->Get_v()) + " ";
	text += "lane = " + to_string(this->Get_lane()) + " ";
	text += "is_inside_lane = " + to_string(this->Get_is_inside_lane()) + "\n";
	text += DISPLAY_PREFIX + "this->trajectory =\n" + this->Get_trajectory().CreateString();
	
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