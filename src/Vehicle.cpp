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
	double theta_start = 0.0;
	vector<double> svdv_start;
	double s_start = 0.0;
	double d_start = 0.0;
	vector<double> svdv_end;
	double sv_start = 0.0;
	double dv_start = 0.0;
	
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
	
	// determine Frenet positions and velocities
	cout << "DEBUG: VU1" << endl;
	theta_start = this->Get_theta();
	cout << "DEBUG: VU2" << endl;
	svdv_start = map.Xy2Frenet(this->Get_x(), this->Get_y(), theta_start);
	cout << "DEBUG: VU3" << endl;
	s_start = svdv_start[0];
	cout << "DEBUG: VU4" << endl;
	d_start = svdv_start[1];
	cout << "DEBUG: VU5" << endl;
	svdv_end = map.Xy2Frenet((this->Get_x() + this->Get_vx()), (this->Get_y() + this->Get_vy()), theta_start);
	cout << "DEBUG: VU6" << endl;
	sv_start = svdv_end[0] - s_start;
	cout << "DEBUG: VU7" << endl;
	dv_start = svdv_end[1] - d_start;
	
	// predict future trajectory
	this->PredictTrajectory(map, s_start, sv_start, d_start, dv_start, theta_start);
	
	// determine values
	this->Update();
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_UPDATE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  theta_start: " << theta_start << endl;
		cout << "  s_start: " << s_start << endl;
		cout << "  d_start: " << d_start << endl;
		cout << "  sv_start: " << sv_start << endl;
		cout << "  dv_start: " << dv_start << endl;
		if (bDISPLAY_VEHICLES) {
			
			cout << "  this: " << endl << this->CreateString();
			
		}
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
		if (bDISPLAY_VEHICLES) {
			
			cout << "  this: " << endl << this->CreateString();
			
		}
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
		if (bDISPLAY_VEHICLES) {
			
			cout << "  this: " << endl << this->CreateString();
			
		}
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
	unsigned int intended_lane = 0;
	
	// initialize outputs
	Trajectory predicted_trajectory;
	
	// add current vehicle position at next step to trajectory
	s_next = map.AssignS(s_start + (sv_start * SAMPLE_TIME));
	sv_next = sv_start;
	sa_next = (sv_next / SAMPLE_TIME);
	sj_next = (sa_next / SAMPLE_TIME);
	d_next = d_start;
	xy_next = map.Frenet2Xy(s_next, d_next);
	x_next = xy_next[0];
	y_next = xy_next[1];
	intended_lane = this->GetLaneD(d_next);
	predicted_trajectory.Start(x_next, y_next, s_next, sv_next, sa_next, sj_next, d_next, dv_start, da_start, dj_start, theta_start, intended_lane);
	
	// determine target values
	sv_target = sv_start;
	//s_target = map.AssignS(s_start + (Average((vector<double>){sv_target, sv_start})) * STEP_TIME_INTERVAL);
	s_target = map.AssignS(s_start + (sv_target * STEP_TIME_INTERVAL));
	dv_target = dv_start;
	//d_target = d_start + (Average((vector<double>){dv_target, dv_start}) * STEP_TIME_INTERVAL);
	d_target = d_start + (dv_target * STEP_TIME_INTERVAL);
	
	// add jerk minimizing trajectory
	predicted_trajectory.AddJerkMinimizingTrajectory(map, s_target, sv_target, sa_target, d_target, dv_target, da_target);
	
	// prevent to reinitialize this trajectory
	predicted_trajectory.Set_is_initialized(true);
	
	// use predicted trajectory for current vehicle
	this->Set_trajectory(predicted_trajectory);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_PREDICTTRAJECTORY) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  s_target: " << s_target << endl;
		cout << "  sv_target: " << sv_target << endl;
		cout << "  d_target: " << d_target << endl;
		cout << "  dv_target: " << dv_target << endl;
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  this->trajectory: " << endl << this->Get_trajectory().CreateString();
			
		}
		cout << "--- VEHICLE: PredictTrajectory - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}
Trajectory Vehicle::PredictTrajectory(Map map) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_PREDICTTRAJECTORY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: PredictTrajectory - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		
	}
	
	// generate prediction with current vehicle's trajectory
	this->PredictTrajectory(map, this->Get_trajectory().Get_s()[0], this->Get_trajectory().Get_sv()[0], this->Get_trajectory().Get_d()[0], this->Get_trajectory().Get_dv()[0], this->Get_trajectory().Get_theta()[0]);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_PREDICTTRAJECTORY) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  this->trajectory: " << endl << this->Get_trajectory().CreateString();
			
		}
		cout << "--- VEHICLE: PredictTrajectory - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return this->Get_trajectory();
	
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
	unsigned int lane = this->DetermineLane(this->Get_d());
	
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
	is_inside_lane = this->CheckInsideLane(this->Get_d(), this->Get_lane());
	
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
vector<Vehicle> Vehicle::Ahead(Map map, const vector<Vehicle> &vehicles, const unsigned int &lane) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_AHEAD) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: Ahead - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles: " << endl << this->CreateVehiclesVectorString(vehicles);
			
		}
		cout << "  lane: " << lane << endl;
		
	}
	
	// initialize outputs
	vector<Vehicle> vehicles_ahead;
	
	// check all vehicles
	for (const Vehicle &vehicle : vehicles) {
		
		// vehicle is in given lane and in front of own vehicle
		if ((vehicle.lane == lane) && (map.DeltaS(vehicle.s, this->Get_s()) > 0)) {
			
			vehicles_ahead.push_back(vehicle);
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_AHEAD) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles_ahead: " << endl << this->CreateVehiclesVectorString(vehicles_ahead);
			
		}
		cout << "--- VEHICLE: Ahead - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return vehicles_ahead;
}

// get vehicles behind own vehicle
vector<Vehicle> Vehicle::Behind(Map map, const vector<Vehicle> &vehicles, const unsigned int &lane) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_BEHIND) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: Behind - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles: " << endl << this->CreateVehiclesVectorString(vehicles);
			
		}
		cout << "  lane: " << lane << endl;
		
	}
	
	// initialize outputs
	vector<Vehicle> vehicles_behind;
	
	// check all vehicles
	for (const Vehicle &vehicle : vehicles) {
		
		// vehicle is in given lane and behind own vehicle
		if ((vehicle.lane == lane) && (map.DeltaS(vehicle.s, this->Get_s()) < 0)) {
			
			vehicles_behind.push_back(vehicle);
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_BEHIND) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles_behind: " << endl << this->CreateVehiclesVectorString(vehicles_behind);
			
		}
		cout << "--- VEHICLE: Behind - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return vehicles_behind;
}

// detect collision between trajectory of own vehicle and trajectories of other vehicles
unsigned long Vehicle::DetectCollision(Map map, Trajectory trajectory, vector<Vehicle> vehicles) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_DETECTCOLLISION) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: DetectCollision - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  trajectory: " << endl << trajectory.CreateString();
			
		}
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(vehicles);
			
		}
		
	}
	
	// define variables
	unsigned long count_t = 0;
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
	unsigned long collision_step = std::numeric_limits<unsigned long>::max();
	
	// investigate all trajectory steps
	for (count_t = 0; count_t < (unsigned long)min(trajectory.Get_s().size(), vehicles[0].Get_trajectory().Get_s().size()); count_t++) {
		
		// get corners of safety box around own vehicle following trajectory
		ego_front_left_s = trajectory.Get_s()[count_t] + SAFETY_BOX_DISTANCE;
		ego_front_left_d = trajectory.Get_d()[count_t] - (this->Get_width() / 2) - SAFETY_BOX_DISTANCE;
		ego_front_right_s = ego_front_left_s;
		ego_front_right_d = trajectory.Get_d()[count_t] + (this->Get_width() / 2) + SAFETY_BOX_DISTANCE;
		ego_rear_right_s = trajectory.Get_s()[count_t] - this->Get_length() - SAFETY_BOX_DISTANCE;
		ego_rear_right_d = ego_front_right_d;
		ego_rear_left_s = ego_rear_right_s;
		ego_rear_left_d = ego_front_left_d;
		
		// check all vehicles
		for (count_v = 0; count_v < vehicles.size(); count_v++) {
			
			// get corners of current vehicle
			vehicle_front_left_s = vehicles[count_v].Get_trajectory().Get_s()[count_t];
			vehicle_front_left_d = vehicles[count_v].Get_trajectory().Get_d()[count_t] - (vehicles[count_v].Get_width() / 2);
			vehicle_front_right_s = vehicle_front_left_s;
			vehicle_front_right_d = vehicles[count_v].Get_trajectory().Get_d()[count_t] + (vehicles[count_v].Get_width() / 2);
			vehicle_rear_right_s = vehicles[count_v].Get_trajectory().Get_s()[count_t] - vehicles[count_v].Get_length();
			vehicle_rear_right_d = vehicle_front_right_d;
			vehicle_rear_left_s = vehicle_rear_right_s;
			vehicle_rear_left_d = vehicle_front_left_d;
			
			// check whether both rectangles overlap
			if (((map.DeltaS(ego_front_left_s, vehicle_rear_right_s) >= 0) && (map.DeltaS(vehicle_front_left_s, ego_rear_right_s) >= 0)) && ((ego_front_left_d <= vehicle_rear_right_d) && (vehicle_front_left_d <= ego_rear_right_d))) {
				
				// collision detected
				collision_step = count_t;
				break; // for count_v loop
				
			}
			
		}
		
		// collision detected
		if (collision_step == count_t) {
			
			break; // for count_t loop
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_DETECTCOLLISION) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  collision_step: " << collision_step << endl;
		cout << "--- VEHICLE: DetectCollision - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return collision_step;
	
}
unsigned long Vehicle::DetectCollision(Map map, vector<Vehicle> vehicles) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_DETECTCOLLISION) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: DetectCollision - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(vehicles);
			
		}
		
	}
	
	// initialize outputs
	unsigned long collision_step = std::numeric_limits<unsigned long>::max();
	
	// check collision with own vehicle's trajectory
	collision_step = this->DetectCollision(map, this->Get_trajectory(), vehicles);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_DETECTCOLLISION) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  collision_step: " << collision_step << endl;
		cout << "--- VEHICLE: DetectCollision - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return collision_step;
	
}

// determine collision cost
double Vehicle::CostStepsToCollision(Map map, Trajectory trajectory, vector<Vehicle> vehicles, const double &weight) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_COSTSTEPSTOCOLLISION) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: CostStepsToCollision - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  trajectory: " << endl << trajectory.CreateString();
			
		}
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(vehicles);
			
		}
		cout << "  weight: " << weight << endl;
		
	}
	
	// define variables
	unsigned long collision_steps = 0;
	double cost_exp = 0.0;
	
	// initialize outputs
	double cost = ZERO_COST;
	
	// check for collision and adjust cost
	collision_steps = DetectCollision(map, trajectory, vehicles);
	
	// calculate cost
	
	// aggressive driver
	cost_exp = exp((NO_HARMFUL_COLLISION_STEPS - collision_steps) / COST_STEPS_TO_COLLISION_SHAPE_FACTOR);
	cost = cost_exp / (cost_exp + 1);
	
	// less aggressive driver
	// cost = weight * (-(collision_steps - NO_HARMFUL_COLLISION_STEPS) / ((COST_STEPS_TO_COLLISION_SHAPE_FACTOR * collision_steps) + NO_HARMFUL_COLLISION_STEPS));
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_COSTSTEPSTOCOLLISION) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  collision_steps: " << collision_steps << endl;
		cout << "  cost: " << cost << endl;
		cout << "--- VEHICLE: CostStepsToCollision - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return cost;
	
}

// determine whether there is enough space to the vehicle in front
double Vehicle::CostSpaceAhead(Map map, Trajectory trajectory, vector<Vehicle> vehicles, const double &weight) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_COSTSPACEAHEAD) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: CostSpaceAhead - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  trajectory: " << endl << trajectory.CreateString();
			
		}
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(vehicles);
			
		}
		cout << "  weight: " << weight << endl;
		
	}
	
	// define variables
	vector<Vehicle> vehicles_ahead;
	unsigned int count = 0;
	Vehicle current_vehicle;
	double current_back_of_vehicle = 0.0;
	double trajectory_distance_to_current_vehicle = 0.0;
	double travel_distance = 0.0;
	double distance_to_current_vehicle = 0.0;
	double minimum_distance_ahead = std::numeric_limits<double>::max();
	Vehicle vehicle_ahead;
	double desired_distance = 0.0;
	double cost_exp = 0.0;
	
	// initialize outputs
	double cost = ZERO_COST;
	
	// get vehicles in front of own vehicle in intended lane
	vehicles_ahead = this->Ahead(map, vehicles, trajectory.Get_intended_lane());
	
	// determine vehicle directly in front of own vehicle
	for (count = 0; count < vehicles_ahead.size(); count++) {
		
		// get current vehicle
		current_vehicle = vehicles_ahead[count];
		
		// calculate distance from end of trajectory to current vehicle
		current_back_of_vehicle = map.DeltaS(current_vehicle.Get_s(), current_vehicle.Get_length());
		cout << "DEBUG trajectory s_end: " << trajectory.Get_s()[trajectory.Get_s().size() - 1]; // TODO remove
		trajectory_distance_to_current_vehicle = map.DeltaS(current_back_of_vehicle, trajectory.Get_s()[trajectory.Get_s().size() - 1]);
		
		// project future distance on current vehicle position
		travel_distance = this->Get_v() * STEP_TIME_INTERVAL;
		distance_to_current_vehicle = trajectory_distance_to_current_vehicle + travel_distance;
		
		// check whether distance is smaller than minimum distance
		if (distance_to_current_vehicle < minimum_distance_ahead) {
			
			// remember this distance as minimum distance
			minimum_distance_ahead = distance_to_current_vehicle;
			vehicle_ahead = current_vehicle;
			
		}
		
	}
	
	// calculate desired distance of end of trajectory to vehicle in front of own vehicle in intended lane
	desired_distance = this->Get_v() * AHEAD_DISTANCE_TIME;
	
	// calculate cost
	cost_exp = exp((desired_distance - minimum_distance_ahead) / COST_SPACE_AHEAD_SHAPE_FACTOR);
	cost = cost_exp / (cost_exp + 1);
	
	// display message if required
	if (bDISPLAY_VEHICLE_COSTSPACEAHEAD) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles_ahead: " << endl << vehicles_ahead[0].CreateVehiclesVectorString(vehicles_ahead);
			
		}
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles_ahead: " << endl << vehicles_ahead[0].CreateVehiclesVectorString(vehicles_ahead);
			
		}
		cout << "  minimum_distance_ahead: " << minimum_distance_ahead << endl;
		cout << "  desired_distance: " << desired_distance << endl;
		cout << "  cost: " << cost << endl;
		cout << "--- VEHICLE: CostSpaceAhead - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return cost;
	
}

// determine whether there is enough space in the intended lane
double Vehicle::CostSpaceInIntendedLane(Map map, Trajectory trajectory, vector<Vehicle> vehicles, const double &weight) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_COSTSPACEININTENDEDLANE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: CostSpaceInIntendedLane - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  trajectory: " << endl << trajectory.CreateString();
			
		}
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(vehicles);
			
		}
		cout << "  weight: " << weight << endl;
		
	}
	
	// define variables
	vector<Vehicle> vehicles_ahead;
	vector<Vehicle> vehicles_behind;
	unsigned int count = 0;
	Vehicle current_vehicle;
	double distance_to_current_vehicle = 0.0;
	double minimum_distance_ahead = std::numeric_limits<double>::max();
	Vehicle vehicle_ahead;
	double minimum_distance_behind = std::numeric_limits<double>::max();
	Vehicle vehicle_behind;
	bool enough_space = false;
	
	// initialize outputs
	double cost = ZERO_COST;
	
	// check whether there is an intended lane change
	if (!(trajectory.Get_intended_lane() == this->Get_lane())) {
		
		// get vehicles in front and behind of own vehicle in intended lane
		vehicles_ahead = this->Ahead(map, vehicles, trajectory.Get_intended_lane());
		vehicles_behind = this->Behind(map, vehicles, trajectory.Get_intended_lane());
		
		// determine vehicle directly in front of own vehicle
		for (count = 0; count < vehicles_ahead.size(); count++) {
			
			// get current vehicle
			current_vehicle = vehicles_ahead[count];
			
			// calculate distance to current vehicle
			distance_to_current_vehicle = map.DeltaS(current_vehicle.Get_s(), this->Get_s());
			
			// check whether distance is smaller than minimum distance
			if (distance_to_current_vehicle < minimum_distance_ahead) {
				
				// remember this distance as minimum distance
				minimum_distance_ahead = distance_to_current_vehicle;
				vehicle_ahead = current_vehicle;
				
			}
			
		}
		
		// determine vehicle directly behind of own vehicle
		for (count = 0; count < vehicles_behind.size(); count++) {
			
			// get current vehicle
			current_vehicle = vehicles_behind[count];
			
			// calculate distance to current vehicle
			distance_to_current_vehicle = map.DeltaS(this->Get_s(), current_vehicle.Get_s());
			
			// check whether distance is smaller than minimum distance
			if (distance_to_current_vehicle < minimum_distance_behind) {
				
				// remember this distance as minimum distance
				minimum_distance_behind = distance_to_current_vehicle;
				vehicle_behind = current_vehicle;
				
			}
			
		}
		
		// determine space needed
		enough_space = ((minimum_distance_ahead >= (AHEAD_SPACE_FACTOR * vehicle_ahead.Get_length())) && (minimum_distance_behind >= (BEHIND_SPACE_FACTOR * vehicle_behind.Get_length())));
		
		// calculate cost
		if (enough_space) {
			
			cost = ZERO_COST;
			
		} else {
			
			cost = weight * MAX_NORMALIZED_COST;
			
		}
		
	}
	
	// display message if required
	if (bDISPLAY_VEHICLE_COSTSPACEININTENDEDLANE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles_ahead: " << endl << vehicles_ahead[0].CreateVehiclesVectorString(vehicles_ahead);
			
		}
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles_behind: " << endl << vehicles_ahead[0].CreateVehiclesVectorString(vehicles_behind);
			
		}
		cout << "  minimum_distance_ahead: " << minimum_distance_ahead << endl;
		cout << "  minimum_distance_behind: " << minimum_distance_behind << endl;
		cout << "  enough_space: " << enough_space << endl;
		cout << "  cost: " << cost << endl;
		cout << "--- VEHICLE: CostSpaceInIntendedLane - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return cost;
	
}

// determine cost for speed in intended lane
double Vehicle::CostSpeedInIntendedLane(Map map, Trajectory trajectory, vector<Vehicle> vehicles, const double &weight) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_COSTSPEEDININTENDEDLANE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: CostSpeedInIntendedLane - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  trajectory: " << endl << trajectory.CreateString();
			
		}
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(vehicles);
			
		}
		cout << "  weight: " << weight << endl;
		
	}
	
	// define variables
	vector<Vehicle> vehicles_ahead;
	unsigned int count = 0;
	Vehicle current_vehicle;
	double distance_to_current_vehicle = 0.0;
	double minimum_distance = std::numeric_limits<double>::max();
	Vehicle vehicle_ahead;
	double lane_speed = 0.0;
	
	// initialize outputs
	double cost = ZERO_COST;
	
	// get vehicles in front of own vehicle in intended lane
	vehicles_ahead = this->Ahead(map, vehicles, trajectory.Get_intended_lane());
	
	// determine vehicle directly in front of own vehicle
	for (count = 0; count < vehicles_ahead.size(); count++) {
		
		// get current vehicle
		current_vehicle = vehicles_ahead[count];
		
		// calculate distance to current vehicle
		distance_to_current_vehicle = map.DeltaS(current_vehicle.Get_s(), this->Get_s());
		
		// check whether distance is smaller than minimum distance
		if (distance_to_current_vehicle < minimum_distance) {
			
			// remember this distance as minimum distance
			minimum_distance = distance_to_current_vehicle;
			vehicle_ahead = current_vehicle;
			
		}
		
	}
	
	// get speed of intended lane
	if (minimum_distance > VEHICLE_AHEAD_WITHIN_DISTANCE) {
		
		lane_speed = MAX_SPEED;
		
	} else {
		
		lane_speed = min(vehicle_ahead.Get_v(), MAX_SPEED);
		
	}
	
	// calculate cost
	cost = weight * (-(lane_speed - MAX_SPEED) / ((COST_SPEED_IN_INTENDED_LANE_SHAPE_FACTOR * lane_speed) + MAX_SPEED));
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_COSTSPEEDININTENDEDLANE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles_ahead: " << endl << vehicles_ahead[0].CreateVehiclesVectorString(vehicles_ahead);
			
		}
		cout << "  minimum_distance: " << minimum_distance << endl;
		cout << "  lane_speed: " << lane_speed << endl;
		cout << "  cost: " << cost << endl;
		cout << "--- VEHICLE: CostSpeedInIntendedLane - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return cost;
	
}

// determine cost for travel distance
double Vehicle::CostTravelDistance(Map map, Trajectory trajectory, const double &weight) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_COSTTRAVELDISTANCE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: CostTravelDistance - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  trajectory: " << endl << trajectory.CreateString();
			
		}
		cout << "  weight: " << weight << endl;
		
	}
	
	// define variables
	double travel_distance = 0.0;
	
	// initialize outputs
	double cost = ZERO_COST;
	
	// calculate travel distance
	travel_distance = map.DeltaS(trajectory.Get_s()[trajectory.Get_s().size() - 1], this->Get_s());
	
	// calculate cost
	cost = weight * (-(travel_distance - MAX_TRAVEL_DISTANCE) / ((COST_TRAVEL_DISTANCE_SHAPE_FACTOR * travel_distance) + MAX_TRAVEL_DISTANCE));
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_COSTTRAVELDISTANCE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  travel_distance: " << travel_distance << endl;
		cout << "  cost: " << cost << endl;
		cout << "--- VEHICLE: CostTravelDistance - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return cost;
	
}

// determine cost of trajectory for own vehicle
	double Vehicle::TrajectoryCost(Map map, Trajectory trajectory, vector<Vehicle> vehicles) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_TRAJECTORYCOST) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "VEHICLE: TrajectoryCost - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  trajectory: " << endl << trajectory.CreateString();
			
		}
		if (bDISPLAY_VEHICLES) {
			
			cout << "  vehicles: " << endl << vehicles[0].CreateVehiclesVectorString(vehicles);
			
		}
		
	}
	
	// define variables
	double cost_steps_to_collision = ZERO_COST;
	double cost_space_ahead = ZERO_COST;
	double cost_space_in_intended_lane = ZERO_COST;
	double cost_speed_in_intended_lane = ZERO_COST;
	double cost_travel_distance = ZERO_COST;
	
	// initialize outputs
	double cost = ZERO_COST;
	
	// add collision cost
	cost_steps_to_collision = this->CostStepsToCollision(map, trajectory, vehicles, COST_COLLISON_WEIGHT);
	cost += cost_steps_to_collision;
	
	// add space ahead cost
	cost_space_ahead = this->CostSpaceAhead(map, trajectory, vehicles, COST_SPACEAHEAD_WEIGHT);
	cost += cost_space_ahead;
	
	// add space in intended lane cost
	cost_space_in_intended_lane = this->CostSpaceInIntendedLane(map, trajectory, vehicles, COST_SPACEININTENDEDLANE_WEIGHT);
	cost += cost_space_in_intended_lane;
	
	// add speed in intended lane cost
	cost_speed_in_intended_lane = this->CostSpeedInIntendedLane(map, trajectory, vehicles, COST_SPEEDININTENDEDLANE_WEIGHT);
	cost += cost_speed_in_intended_lane;
	
	// add travel distance cost
	cost_travel_distance = this->CostTravelDistance(map, trajectory, COST_TRAVELDISTANCE_WEIGHT);
	cost += cost_travel_distance;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_VEHICLE_TRAJECTORYCOST) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  cost_steps_to_collision: " << cost_steps_to_collision << endl;
		cout << "  cost_space_ahead: " << cost_space_ahead << endl;
		cout << "  cost_space_in_intended_lane: " << cost_space_in_intended_lane << endl;
		cout << "  cost_speed_in_intended_lane: " << cost_speed_in_intended_lane << endl;
		cout << "  cost_travel_distance: " << cost_travel_distance << endl;
		cout << "  cost: " << cost << endl;
		cout << "--- VEHICLE: TrajectoryCost - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return cost;
	
}

// set trajectory
void Vehicle::Set_trajectory(Trajectory trajectory) {
	
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
	if (bDISPLAY_TRAJECTORIES) {
		
		text += DISPLAY_PREFIX + "this->trajectory =\n" + this->Get_trajectory().CreateString();
		
	}
	
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