/*
 * Trajectory.cpp
 *
 * Trajectory class
 *
 * Created on 01/04/2019
 * Author: Andre Strobel
 *
 */

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <cmath>
#include "Trajectory.h"
#include "Map.h"
#include "Vehicle.h"
#include "Path.h"
#include "helper_functions.h"

using std::vector;
using std::string;
using std::to_string;
using std::cout;
using std::endl;
using std::pow;

// init trajectory
unsigned long Trajectory::Init(Map map, Vehicle ego, Path previous_path) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_INIT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: Init - Start" << endl;
		cout << "  ego: " << endl << ego.CreateString();
		cout << "  previous_path: " << endl << previous_path.CreateString();
		
	}
	
	// initialize outputs
	unsigned long finished_steps = 0;
	double s_next = 0.0;
	double sv_next = 0.0;
	double sa_next = 0.0;
	double sj_next = 0.0;
	double d_next = 0.0;
	vector<double> xy_next;
	double x_next = 0.0;
	double y_next = 0.0;
	
	// determine how much progress was made since last call
	finished_steps = (unsigned long)(this->x_values.size() - previous_path.Get_x().size());
		
	// check whether this trajectory has not been initialized or whether it has not enough steps left or whether it has no unused steps left
	if ((!this->is_initialized) || (this->Get_x().size() < MIN_PREVIOUS_PATH_STEPS) || (this->Get_x().size() <= (finished_steps + 1))) {
		
		// add own vehicle position at next step to trajectory
		s_next = ego.Get_s() + (EGO_CAR_SV_INIT * SAMPLE_TIME);
		sv_next = EGO_CAR_SV_INIT;
		sa_next = (sv_next / SAMPLE_TIME);
		sj_next = (sa_next / SAMPLE_TIME);
		d_next = ego.Get_d();
		xy_next = map.Frenet2Xy(s_next, d_next);
		x_next = xy_next[0];
		y_next = xy_next[1];
		this->Start(x_next, y_next, s_next, sv_next, sa_next, sj_next, d_next, EGO_CAR_DV_INIT, EGO_CAR_DA_INIT, EGO_CAR_DJ_INIT, ego.Get_theta());
		
		// initialization done
		this->is_initialized = true;
		
	} else {
		
		// remove finished steps from existing trajectory
		this->RemoveFirstSteps(finished_steps);
		
		// start with previous trajectory
		this->KeepFirstSteps(NUM_PREVIOUS_PATH_STEPS);
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_INIT) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  finished_steps: " << finished_steps << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- TRAJECTORY: Init - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return finished_steps;
	
}

// start trajectory
void Trajectory::Start(double x, double y, double s, double sv, double sa, double sj, double d, double dv, double da, double dj, double theta) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_START) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: Start - Start" << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << x << endl;
		cout << "  s: " << s << endl;
		cout << "  sv: " << sv << endl;
		cout << "  sa " << sa << endl;
		cout << "  sj: " << sj << endl;
		cout << "  d: " << d << endl;
		cout << "  dv: " << dv << endl;
		cout << "  da: " << da << endl;
		cout << "  dj: " << dj << endl;
		cout << "  theta: " << theta << endl;
		
	}
	
	// start trajectory with this step
	this->x_values = vector<double>{x};
	this->y_values = vector<double>{y};
	this->s_values = vector<double>{s};
	this->sv_values = vector<double>{sv};
	this->sa_values = vector<double>{sa};
	this->sj_values = vector<double>{sj};
	this->d_values = vector<double>{d};
	this->dv_values = vector<double>{dv};
	this->da_values = vector<double>{da};
	this->dj_values = vector<double>{dj};
	this->theta_values = vector<double>{theta};
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_START) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values: " << endl << CreateDoubleVectorsString((vector<vector<double>>){this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values});
		cout << "--- TRAJECTORY: Start - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// add step to trajectory
void Trajectory::Add(double x, double y, double s, double sv, double sa, double sj, double d, double dv, double da, double dj, double theta) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADD) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: Add - Start" << endl;
		cout << "  x: " << x << endl;
		cout << "  y: " << x << endl;
		cout << "  s: " << s << endl;
		cout << "  sv: " << sv << endl;
		cout << "  sa " << sa << endl;
		cout << "  sj: " << sj << endl;
		cout << "  d: " << d << endl;
		cout << "  dv: " << dv << endl;
		cout << "  da: " << da << endl;
		cout << "  dj: " << dj << endl;
		cout << "  theta: " << theta << endl;
		
	}
	
	// add trajectory step
	this->x_values.push_back(x);
	this->y_values.push_back(y);
	this->s_values.push_back(s);
	this->sv_values.push_back(sv);
	this->sa_values.push_back(sa);
	this->sj_values.push_back(sj);
	this->d_values.push_back(d);
	this->dv_values.push_back(dv);
	this->da_values.push_back(da);
	this->dj_values.push_back(dj);
	this->theta_values.push_back(theta);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADD) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values: " << endl << CreateDoubleVectorsString((vector<vector<double>>){this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values});
		cout << "--- TRAJECTORY: Add - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}
void Trajectory::Add(Trajectory trajectory, unsigned long max_num_steps) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADD) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: Add - Start" << endl;
		cout << "  trajectory: " << endl << trajectory.CreateString();
		cout << "  max_num_steps: " << max_num_steps << endl;
		
	}
	
	// define variables
	unsigned long count = 0;
	
	// add trajectory segment to this trajectory
	for (count = 0; ((count < trajectory.Get_x().size()) && (count < max_num_steps)); count++) {
		
		// add current segment
		this->Add(trajectory.Get_x()[count], trajectory.Get_y()[count], trajectory.Get_s()[count], trajectory.Get_sv()[count], trajectory.Get_sa()[count], trajectory.Get_sj()[count], trajectory.Get_d()[count], trajectory.Get_dv()[count], trajectory.Get_da()[count], trajectory.Get_dj()[count], trajectory.Get_theta()[count]);
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADD) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- TRAJECTORY: Add - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}
void Trajectory::Add(Trajectory trajectory) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADD) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: Add - Start" << endl;
		cout << "  trajectory: " << endl << trajectory.CreateString();
		
	}
	
	// add full trajectory to this trajectory
	this->Add(trajectory, trajectory.Get_x().size());
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADD) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- TRAJECTORY: Add - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// add jerk minimizing trajectory
void Trajectory::AddJerkMinimizingTrajectory(Map map, double s_target, double sv_target, double sa_target, double d_target, double dv_target, double da_target) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADDJERKMINIMIZINGTRAJECTORY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: AddJerkMinimizingTrajectory - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		cout << "  s_target: " << s_target << endl;
		cout << "  sv_target: " << sv_target << endl;
		cout << "  sa_target: " << sa_target << endl;
		cout << "  d_target: " << d_target << endl;
		cout << "  dv_target: " << dv_target << endl;
		cout << "  da_target: " << da_target << endl;
		
	}
	
	// define variables
	unsigned int trajectory_length = 0;
	double x_last = 0.0;
	double y_last = 0.0;
	double s_last = 0.0;
	double sv_last = 0.0;
	double sa_last = 0.0;
	double d_last = 0.0;
	double dv_last = 0.0;
	double da_last = 0.0;
	vector<double> s_start_vector;
	vector<double> s_end_vector;
	vector<double> s_poly;
	vector<double> d_start_vector;
	vector<double> d_end_vector;
	vector<double> d_poly;
	unsigned long remaining_steps = 0;
	unsigned long count = 0;
	double t = 0.0;
	vector<double> s_states;
	double s = 0.0;
	double sv = 0.0;
	double sa = 0.0;
	double sj = 0.0;
	vector<double> d_states;
	double d = 0.0;
	double dv = 0.0;
	double da = 0.0;
	double dj = 0.0;
	vector<double> xy_values;
	double x = 0.0;
	double y = 0.0;
	double theta = 0;
	
	// determine length of trajectory
	trajectory_length = this->Get_x().size();
	
	// use last trajectory values as starting point for new trajectory segment if available
	x_last = this->Get_x()[trajectory_length - 1];
	y_last = this->Get_y()[trajectory_length - 1];
	s_last = this->Get_s()[trajectory_length - 1];
	sv_last = this->Get_sv()[trajectory_length - 1];
	sa_last = this->Get_sa()[trajectory_length - 1];
	d_last = this->Get_d()[trajectory_length - 1];
	dv_last = this->Get_dv()[trajectory_length - 1];
	da_last = this->Get_da()[trajectory_length - 1];
	
	// determine coefficients for jerk minimizing trajectory
	s_start_vector = (vector<double>){s_last, sv_last, sa_last};
	s_end_vector = (vector<double>){s_target, sv_target, sa_target};
	s_poly = JerkMinimizingTrajectoryCoefficients(s_start_vector, s_end_vector, STEP_TIME_INTERVAL);
	d_start_vector = (vector<double>){d_last, dv_last, da_last};
	d_end_vector = (vector<double>){d_target, dv_target, da_target};
	d_poly = JerkMinimizingTrajectoryCoefficients(d_start_vector, d_end_vector, STEP_TIME_INTERVAL);
	
	// determine number of steps to create
	remaining_steps = (STEP_TIME_INTERVAL / SAMPLE_TIME) - trajectory_length;
	
	for (count = 0; count < remaining_steps; count++) {
		
		// determine time value
		t = SAMPLE_TIME * (count + 1);
		
		// determine states
		s_states = JerkMinimizingTrajectoryState(s_poly, s_start_vector, t);
		s = s_states[0];
		sv = s_states[1];
		sa = s_states[2];
		sj = s_states[3];
		d_states = JerkMinimizingTrajectoryState(d_poly, d_start_vector, t);
		d = d_states[0];
		dv = d_states[1];
		da = d_states[2];
		dj = d_states[3];
		
		// convert Frenet coordinates to xy coordinates
		xy_values = map.Frenet2Xy(s, d);
		x = xy_values[0];
		y = xy_values[1];
		
		// determine orientation angle
		theta = atan2(y - y_last, x - x_last);
		
		// add new step to trajectory
		this->Add(x, y, s, sv, sa, sj, d, dv, da, dj, theta);
		
		// update last values for next step (to determine orientation angle again)
		x_last = x;
		y_last = y;
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADDJERKMINIMIZINGTRAJECTORY) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  trajectory_length: " << trajectory_length << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- TRAJECTORY: AddJerkMinimizingTrajectory - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// generate new trajectory
void Trajectory::Generate(Map map, Trajectory trajectory, double s_target, double sv_target, double sa_target, double d_target, double dv_target, double da_target) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_GENERATE) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: Generate - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		cout << "  trajectory: " << endl << trajectory.CreateString();
		cout << "  s_target: " << s_target << endl;
		cout << "  sv_target: " << sv_target << endl;
		cout << "  sa_target: " << sa_target << endl;
		cout << "  d_target: " << d_target << endl;
		cout << "  dv_target: " << dv_target << endl;
		cout << "  da_target: " << da_target << endl;
		
	}
	
	// add previous trajectory segment
	this->Add(trajectory);
	
	// add final segment as jerk minimizing trajectory
	this->AddJerkMinimizingTrajectory(map, s_target, sv_target, sa_target, d_target, dv_target, da_target);
	
	// prevent to reinitialize this trajectory
	this->is_initialized = true;
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_GENERATE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- TRAJECTORY: Generate - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// check trajectory for being valid
bool Trajectory::Valid() {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_VALID) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: Valid - Start" << endl;
		
	}
	
	// initialize outputs
	bool is_valid = true;
	
	
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_VALID) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "--- TRAJECTORY: Valid - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return is_valid;
	
}

// determine cost of trajectory
double Trajectory::Cost() {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_COST) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: Cost - Start" << endl;
		
	}
	
	// initialize outputs
	double cost = 0;
	
	
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_COST) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  cost: " << cost << endl;
		cout << "--- TRAJECTORY: Cost - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return cost;
	
}

// get trajectory x values
vector<double> Trajectory::Get_x() {
	
	return this->x_values;
	
}

// get trajectory y values
vector<double> Trajectory::Get_y() {
	
	return this->y_values;
	
}

// get trajectory s values
vector<double> Trajectory::Get_s() {
	
	return this->s_values;
	
}

// get trajectory s velocity values
vector<double> Trajectory::Get_sv() {
	
	return this->sv_values;
	
}

// get trajectory s acceleration values
vector<double> Trajectory::Get_sa() {
	
	return this->sa_values;
	
}

// get trajectory s jerk values
vector<double> Trajectory::Get_sj() {
	
	return this->sj_values;
	
}
// get trajectory d values
vector<double> Trajectory::Get_d() {
	
	return this->d_values;
	
}

// get trajectory d velocity values
vector<double> Trajectory::Get_dv() {
	
	return this->dv_values;
	
}

// get trajectory d acceleration values
vector<double> Trajectory::Get_da() {
	
	return this->da_values;
	
}

// get trajectory d jerk values
vector<double> Trajectory::Get_dj() {
	
	return this->dj_values;
	
}

// get trajectory orientation angles
vector<double> Trajectory::Get_theta() {
	
	return this->theta_values;
	
}

// get initialization status
bool Trajectory::Get_is_initialized() {
	
	return this->is_initialized;
	
}

// display Trajectory object as string
string Trajectory::CreateString() {
	
	//define variables
	string text = "";
	
	// add information about path to string
	text += DISPLAY_PREFIX + "this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values =\n" + CreateDoubleVectorsString((vector<vector<double>>){this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values});
	text += DISPLAY_PREFIX;
	text += "is_initialized = " + to_string(this->is_initialized) + "\n";
	
	// return output
	return text;
	
}

// remove steps from the front
void Trajectory::RemoveFirstSteps(const unsigned long &steps) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_REMOVEFIRSTSTEPS) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: RemoveFirstSteps - Start" << endl;
		cout << "  steps: " << steps << endl;
		
	}
	
	// remove first steps
	this->x_values.erase(this->x_values.begin(), this->x_values.begin() + steps);
	this->y_values.erase(this->y_values.begin(), this->y_values.begin() + steps);
	this->s_values.erase(this->s_values.begin(), this->s_values.begin() + steps);
	this->sv_values.erase(this->sv_values.begin(), this->sv_values.begin() + steps);
	this->sa_values.erase(this->sa_values.begin(), this->sa_values.begin() + steps);
	this->sj_values.erase(this->sj_values.begin(), this->sj_values.begin() + steps);
	this->d_values.erase(this->d_values.begin(), this->d_values.begin() + steps);
	this->dv_values.erase(this->dv_values.begin(), this->dv_values.begin() + steps);
	this->da_values.erase(this->da_values.begin(), this->da_values.begin() + steps);
	this->dj_values.erase(this->dj_values.begin(), this->dj_values.begin() + steps);
	this->theta_values.erase(this->theta_values.begin(), this->theta_values.begin() + steps);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_REMOVEFIRSTSTEPS) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values: " << endl << CreateDoubleVectorsString((vector<vector<double>>){this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values});
		cout << "--- TRAJECTORY: RemoveFirstSteps - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// keep steps from the front
void Trajectory::KeepFirstSteps(const unsigned long &steps) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_KEEPFIRSTSTEPS) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: KeepFirstSteps - Start" << endl;
		cout << "  steps: " << steps << endl;
		
	}
	
	// keep first steps
	this->x_values.erase(this->x_values.begin() + steps, this->x_values.end());
	this->y_values.erase(this->y_values.begin() + steps, this->y_values.end());
	this->s_values.erase(this->s_values.begin() + steps, this->s_values.end());
	this->sv_values.erase(this->sv_values.begin() + steps, this->sv_values.end());
	this->sa_values.erase(this->sa_values.begin() + steps, this->sa_values.end());
	this->sj_values.erase(this->sj_values.begin() + steps, this->sj_values.end());
	this->d_values.erase(this->d_values.begin() + steps, this->d_values.end());
	this->dv_values.erase(this->dv_values.begin() + steps, this->dv_values.end());
	this->da_values.erase(this->da_values.begin() + steps, this->da_values.end());
	this->dj_values.erase(this->dj_values.begin() + steps, this->dj_values.end());
	this->theta_values.erase(this->theta_values.begin() + steps, this->theta_values.end());
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_KEEPFIRSTSTEPS) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values: " << endl << CreateDoubleVectorsString((vector<vector<double>>){this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values});
		cout << "--- TRAJECTORY: KeepFirstSteps - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}