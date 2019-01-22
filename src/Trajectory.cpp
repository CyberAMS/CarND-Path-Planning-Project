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
using std::fmod;
using std::min;
using std::pow;

// init trajectory
unsigned long Trajectory::Init(Map map, double s_start, double sv_start, double d_start, double dv_start, double da_start, double dj_start, double theta_start, double intended_lane, Path previous_path) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_INIT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: Init - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		cout << "  s_start: " << s_start << endl;
		cout << "  sv_start: " << sv_start << endl;
		cout << "  d_start: " << d_start << endl;
		cout << "  dv_start: " << dv_start << endl;
		cout << "  da_start: " << da_start << endl;
		cout << "  dj_start: " << dj_start << endl;
		cout << "  theta_start: " << theta_start << endl;
		cout << "  intended_lane: " << intended_lane << endl;
		cout << "  previous_path: " << endl << previous_path.CreateString();
		
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
	
	// initialize outputs
	unsigned long finished_steps = 0;
	
	// determine how much progress was made since last call
	finished_steps = (unsigned long)(this->Get_x().size() - previous_path.Get_x().size());
		
	// check whether this trajectory has not been initialized or whether it has not enough steps left or whether it has no unused steps left
	if ((!this->Get_is_initialized()) || (this->Get_x().size() < MIN_PREVIOUS_PATH_STEPS) || (this->Get_x().size() <= (finished_steps + 1))) {
		
		// add own vehicle position at next step to trajectory
		s_next = map.AssignS(s_start + (sv_start * SAMPLE_TIME));
		sv_next = sv_start;
		sa_next = (sv_next / SAMPLE_TIME);
		sj_next = (sa_next / SAMPLE_TIME);
		d_next = d_start;
		xy_next = map.Frenet2Xy(s_next, d_next);
		x_next = xy_next[0];
		y_next = xy_next[1];
		this->Start(x_next, y_next, s_next, sv_next, sa_next, sj_next, d_next, dv_start, da_start, dj_start, theta_start, intended_lane);
		
		// initialization done
		this->is_initialized = true;
		
	} else {
		
		// remove finished steps from existing trajectory
		this->RemoveFirstSteps(finished_steps);
		
		// start with previous trajectory
		this->KeepFirstSteps(NUM_PREVIOUS_PATH_STEPS);
		
	}
	
	// remember how many steps have been taken from previous trajectory
	this->previous_trajectory_steps = this->Get_x().size();
	
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
void Trajectory::Start(double x, double y, double s, double sv, double sa, double sj, double d, double dv, double da, double dj, double theta, unsigned int intended_lane) {
	
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
		cout << "  intended_lane: " << intended_lane << endl;
		
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
	
	// set intended lane
	this->Set_intended_lane(intended_lane);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_START) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values: " << endl << CreateDoubleVectorsString((vector<vector<double>>){this->Get_x(), this->Get_y(), this->Get_s(), this->Get_sv(), this->Get_sa(), this->Get_sj(), this->Get_d(), this->Get_dv(), this->Get_da(), this->Get_dj(), this->Get_theta()});
		cout << "  this->intended_lane: " << this->Get_intended_lane() << endl;
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
		cout << "  this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values: " << endl << CreateDoubleVectorsString((vector<vector<double>>){this->Get_x(), this->Get_y(), this->Get_s(), this->Get_sv(), this->Get_sa(), this->Get_sj(), this->Get_d(), this->Get_dv(), this->Get_da(), this->Get_dj(), this->Get_theta()});
		cout << "--- TRAJECTORY: Add - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}
void Trajectory::Add(Trajectory trajectory, unsigned long max_num_steps) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_ADD) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: Add - Start" << endl;
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  trajectory: " << endl << trajectory.CreateString();
			
		}
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
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  trajectory: " << endl << trajectory.CreateString();
			
		}
		
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
	if (trajectory_length > 0) {
		
		x_last = this->Get_x()[trajectory_length - 1];
		y_last = this->Get_y()[trajectory_length - 1];
		s_last = this->Get_s()[trajectory_length - 1];
		sv_last = this->Get_sv()[trajectory_length - 1];
		sa_last = this->Get_sa()[trajectory_length - 1];
		d_last = this->Get_d()[trajectory_length - 1];
		dv_last = this->Get_dv()[trajectory_length - 1];
		da_last = this->Get_da()[trajectory_length - 1];
		
	} else {
		
		// display error message
		cout << "TRAJECTORY: AddJerkMinimizingTrajectory - Error: trajectory_length must be > 0 and is " << trajectory_length << endl;
		
	}
	
	// adjust for loop track
	if (s_target < s_last) {
		
		s_target += MAX_TRACK_S;
		
	}
	
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
		s = map.AssignS(s_states[0]);
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
		if (bDISPLAY_TRAJECTORIES) {
			
			cout << "  trajectory: " << endl << trajectory.CreateString();
			
		}
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
	
	// prevent to reinitialize this trajectory and set all necessary variables
	this->is_initialized = true;
	this->previous_trajectory_steps = trajectory.Get_previous_trajectory_steps();
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_GENERATE) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "--- TRAJECTORY: Generate - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}

// check trajectory for being valid
bool Trajectory::Valid(Map map) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_VALID) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: Valid - Start" << endl;
		// cout << "  map: " << endl << map.CreateString();
		
	}
	
	// define variables
	double max_sv = 0.0;
	double min_sv = 0.0;
	double average_sv = 0.0;
	double max_sa = 0.0;
	double min_sa = 0.0;
	double average_sa = 0.0;
	double max_sj = 0.0;
	double min_sj = 0.0;
	double average_sj = 0.0;
	double max_d = 0.0;
	double min_d = 0.0;
	double max_dv = 0.0;
	double min_dv = 0.0;
	double average_dv = 0.0;
	double max_da = 0.0;
	double min_da = 0.0;
	double average_da = 0.0;
	double max_dj = 0.0;
	double min_dj = 0.0;
	double average_dj = 0.0;
	vector<double> v_values;
	double max_v = 0.0;
	double min_v = 0.0;
	double average_v = 0.0;
	vector<double> a_values;
	double max_a = 0.0;
	double min_a = 0.0;
	double average_a = 0.0;
	vector<double> j_values;
	double max_j = 0.0;
	double min_j = 0.0;
	double average_j = 0.0;
	double gain_sv = NEUTRAL_GAIN;
	double gain_sa = NEUTRAL_GAIN;
	double gain_v = NEUTRAL_GAIN;
	double gain_a = NEUTRAL_GAIN;
	double gain = NEUTRAL_GAIN;
	unsigned long count = 0;
	unsigned long previous_trajectory_steps = 0;
	vector<double> sv_values;
	vector<double> d_values;
	vector<double> new_s_values;
	vector<double> new_sv_values;
	vector<double> new_sa_values;
	vector<double> new_sj_values;
	vector<vector<double>> new_xy_values;
	vector<double> new_x_values;
	vector<double> new_y_values;
	vector<double> new_theta_values;
	double d = 0.0;
	Vehicle vehicle_to_call_fcn;
	
	// initialize outputs
	bool is_valid = true;
	return is_valid; // TODO: eliminated validation
	
	// calculate distance, speed, acceleration and jerk values based on s and d values
	max_sv = Maximum(this->Get_sv());
	min_sv = Minimum(this->Get_sv());
	average_sv = AbsAverage(this->Get_sv());
	max_sa = Maximum(this->Get_sa());
	min_sa = Minimum(this->Get_sa());
	average_sa = AbsAverage(this->Get_sa());
	max_sj = Maximum(this->Get_sj());
	min_sj = Minimum(this->Get_sj());
	average_sj = AbsAverage(this->Get_sj());
	max_d = Maximum(this->Get_d());
	min_d = Minimum(this->Get_d());
	max_dv = Maximum(this->Get_dv());
	min_dv = Minimum(this->Get_dv());
	average_dv = AbsAverage(this->Get_dv());
	max_da = Maximum(this->Get_da());
	min_da = Minimum(this->Get_da());
	average_da = AbsAverage(this->Get_da());
	max_dj = Maximum(this->Get_dj());
	min_dj = Minimum(this->Get_dj());
	average_dj = AbsAverage(this->Get_dj());
	
	// calculate speed, acceleration and jerk values based on xy values
	// TODO: Sometimes the v and a values don't make sense and jump
	v_values = Multiply(Magnitude(Differential(this->Get_x()), Differential(this->Get_y())), (1 / SAMPLE_TIME));
	v_values.erase(v_values.begin(), v_values.begin() + 1); // make sure to not use first values that came from Differential and are ZERO_DIFFERENTIAL_VALUE
	max_v = Maximum(v_values);
	min_v = Minimum(v_values);
	average_v = AbsAverage(v_values);
	a_values = Multiply(Differential(v_values), (1 / SAMPLE_TIME));
	a_values.erase(a_values.begin(), a_values.begin() + 1); // make sure to not use first values that came from Differential and are ZERO_DIFFERENTIAL_VALUE
	max_a = Maximum(a_values);
	min_a = Minimum(a_values);
	average_a = AbsAverage(a_values);
	j_values = Multiply(Differential(a_values), (1 / SAMPLE_TIME));
	j_values.erase(j_values.begin(), j_values.begin() + 1); // make sure to not use first values that came from Differential and are ZERO_DIFFERENTIAL_VALUE
	max_j = Maximum(j_values);
	min_j = Minimum(j_values);
	average_j = AbsAverage(j_values);
	
	// determine gains for trajectory to be conform with maximum longitudinal speed, acceleration and deceleration
	if (max_sv > MAX_SPEED) {
		
		gain_sv = min(gain_sv, (MAX_SPEED / max_sv));
		
	}
	if (max_sa > MAX_ACCELERATION_S) {
		
		gain_sa = min(gain_sa, (MAX_ACCELERATION_S / max_sa));
		
	}
	if (min_sa < MAX_DECELERATION_S) {
		
		gain_sa = min(gain_sa, (MAX_DECELERATION_S / min_sa));
		
	}
	if (max_v > MAX_SPEED) {
		
		gain_v = min(gain_v, (MAX_SPEED / max_v));
		
	}
	if (max_a > MAX_ACCELERATION_S) {
		
		gain_a = min(gain_a, (MAX_ACCELERATION_S / max_a));
		
	}
	if (min_a < MAX_DECELERATION_S) {
		
		gain_a = min(gain_a, (MAX_DECELERATION_S / min_a));
		
	}
	
	// select gain
	switch (TRAJECTORY_VALID_GAIN_SELECTION) {
		
		case NOTHING:
			
			// use no gains
			gain = NEUTRAL_GAIN;
			break; // switch
			
		case SV_V:
			
			// only use velocity gains
			gain = min(gain_sv, gain_v);
			break; // switch
			
		case SV_SA:
			
			// only use sd gains
			gain = min(gain_sv, gain_sa);
			break; // switch
			
		case SV_SA_V:
			
			// only use sd gains and velocity
			gain = min(min(gain_sv, gain_sa), gain_v);
			break; // switch
			
		case ALL:
			
			// use all gains
			gain = min(min(min(gain_sv, gain_sa), gain_v), gain_a);
			break; // switch
			
	}
	
	// apply gain if necessary
	if (gain < NEUTRAL_GAIN) {
		
		// determine length of previous trajectory segment
		previous_trajectory_steps = this->Get_previous_trajectory_steps();
		
		// select new segment only
		for (count = previous_trajectory_steps; count < this->Get_sv().size(); count++) {
			
			// save current step
			sv_values.push_back(this->Get_sv()[count]);
			d_values.push_back(this->Get_d()[count]);
			
		}
		
		// adjust speed of new segment
		new_sv_values = Multiply(sv_values, gain);
		
		// generate distance, acceleration and jerk of new segment
		new_s_values = map.AssignS(Addition(Accumulate(Multiply(new_sv_values, SAMPLE_TIME)), this->Get_s()[previous_trajectory_steps - 1]));
		new_sa_values = Multiply(Differential(new_sv_values), (1 / SAMPLE_TIME));
		new_sj_values = Multiply(Differential(new_sa_values), (1 / SAMPLE_TIME));
		
		// regenerate xy values including theta of new segment
		new_xy_values = map.Frenet2Xy(new_s_values, d_values);
		new_x_values = new_xy_values[0];
		new_y_values = new_xy_values[1];
		new_theta_values = Angle(Differential(new_x_values), Differential(new_y_values));
		
		// update new segment only
		for (count = previous_trajectory_steps; count < this->Get_sv().size(); count++) {
			
			// make sure to not use first values that came from Differential and are ZERO_DIFFERENTIAL_VALUE
			if (count == previous_trajectory_steps) {
				
				// update only valid values of current step (not first or higher Differential)
				this->s_values[count] = new_s_values[count - previous_trajectory_steps];
				this->sv_values[count] = new_sv_values[count - previous_trajectory_steps];
				this->x_values[count] = new_x_values[count - previous_trajectory_steps];
				this->y_values[count] = new_y_values[count - previous_trajectory_steps];
				
			} else if (count == (previous_trajectory_steps + 1)) {
				
				// update only valid values of current step (no second Differential)
				this->s_values[count] = new_s_values[count - previous_trajectory_steps];
				this->sv_values[count] = new_sv_values[count - previous_trajectory_steps];
				this->sa_values[count] = new_sa_values[count - previous_trajectory_steps];
				this->x_values[count] = new_x_values[count - previous_trajectory_steps];
				this->y_values[count] = new_y_values[count - previous_trajectory_steps];
				this->theta_values[count] = new_theta_values[count - previous_trajectory_steps];
				
			} else {
				
				// update current step
				this->s_values[count] = new_s_values[count - previous_trajectory_steps];
				this->sv_values[count] = new_sv_values[count - previous_trajectory_steps];
				this->sa_values[count] = new_sa_values[count - previous_trajectory_steps];
				this->sj_values[count] = new_sj_values[count - previous_trajectory_steps];
				this->x_values[count] = new_x_values[count - previous_trajectory_steps];
				this->y_values[count] = new_y_values[count - previous_trajectory_steps];
				this->theta_values[count] = new_theta_values[count - previous_trajectory_steps];
				
			}
			
		}
		
	}
	
	// check whether lateral trajectory accelerations are valid
	if ((max_da > MAX_ACCELERATION_D) || (min_da < -MAX_ACCELERATION_D)) {
		
		is_valid = false;
		
	}
	
	// check whether trajectory stays in lane
	d = this->Get_d()[this->Get_d().size() - 1];
	if (!vehicle_to_call_fcn.CheckInsideLane(d, vehicle_to_call_fcn.DetermineLane(d))) {
		
		is_valid = false;
		
	}
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_VALID) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  max_sv: " << max_sv << endl;
		cout << "  min_sv: " << min_sv << endl;
		cout << "  average_sv: " << average_sv << endl;
		cout << "  max_sa: " << max_sa << endl;
		cout << "  min_sa: " << min_sa << endl;
		cout << "  average_sa: " << average_sa << endl;
		cout << "  max_sj: " << max_sj << endl;
		cout << "  min_sj: " << min_sj << endl;
		cout << "  average_sj: " << average_sj << endl;
		cout << "  max_d: " << max_d << endl;
		cout << "  min_d: " << min_d << endl;
		cout << "  max_dv: " << max_dv << endl;
		cout << "  min_dv: " << min_dv << endl;
		cout << "  average_dv: " << average_dv << endl;
		cout << "  max_da: " << max_da << endl;
		cout << "  min_da: " << min_da << endl;
		cout << "  average_da: " << average_da << endl;
		cout << "  max_dj: " << max_dj << endl;
		cout << "  min_dj: " << min_dj << endl;
		cout << "  average_dj: " << average_dj << endl;
		//cout << "  v_values, a_values, j_values: " << endl << CreateDoubleVectorsString((vector<vector<double>>){v_values, a_values, j_values});
		//cout << "  v_values: " << endl << CreateDoubleVectorString(v_values);
		//cout << "  a_values: " << endl << CreateDoubleVectorString(a_values);
		//cout << "  j_values: " << endl << CreateDoubleVectorString(j_values);
		cout << "  max_v: " << max_v << endl;
		cout << "  min_v: " << min_v << endl;
		cout << "  average_v: " << average_v << endl;
		cout << "  max_a: " << max_a << endl;
		cout << "  min_a: " << min_a << endl;
		cout << "  average_a: " << average_a << endl;
		cout << "  max_j: " << max_j << endl;
		cout << "  min_j: " << min_j << endl;
		cout << "  average_j: " << average_j << endl;
		cout << "  gain_sv: " << gain_sv << endl;
		cout << "  gain_sa: " << gain_sa << endl;
		cout << "  gain_v: " << gain_v << endl;
		cout << "  gain_a: " << gain_a << endl;
		cout << "  gain: " << gain << endl;
		//cout << "  sv_values, d_values: " << endl << CreateDoubleVectorsString((vector<vector<double>>){sv_values, d_values});
		//cout << "  new_s_values, new_sv_values, new_sa_values, new_sj_values, new_x_values, new_y_values, new_theta_values: " << endl << CreateDoubleVectorsString((vector<vector<double>>){new_s_values, new_sv_values, new_sa_values, new_sj_values, new_x_values, new_y_values, new_theta_values});
		cout << "  d: " << d << endl;
		cout << "  this: " << endl << this->CreateString();
		cout << "  is_valid: " << is_valid << endl;
		cout << "--- TRAJECTORY: Valid - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	return is_valid;
	
}

// set intended lane
void Trajectory::Set_intended_lane(unsigned int intended_lane) {
	
	this->intended_lane = intended_lane;
	
}

// set initialization status
void Trajectory::Set_is_initialized(bool is_initialized) {
	
	this->is_initialized = is_initialized;
	
}

// get trajectory x values
vector<double> Trajectory::Get_x() {
	
	return this->x_values;
	
}
vector<double>* Trajectory::Get_x_ptr() {
	
	return &this->x_values;
	
}

// get trajectory y values
vector<double> Trajectory::Get_y() {
	
	return this->y_values;
	
}
vector<double>* Trajectory::Get_y_ptr() {
	
	return &this->y_values;
	
}

// get trajectory s values
vector<double> Trajectory::Get_s() {
	
	return this->s_values;
	
}
vector<double>* Trajectory::Get_s_ptr() {
	
	return &this->s_values;
	
}

// get trajectory s velocity values
vector<double> Trajectory::Get_sv() {
	
	return this->sv_values;
	
}
vector<double>* Trajectory::Get_sv_ptr() {
	
	return &this->sv_values;
	
}

// get trajectory s acceleration values
vector<double> Trajectory::Get_sa() {
	
	return this->sa_values;
	
}
vector<double>* Trajectory::Get_sa_ptr() {
	
	return &this->sa_values;
	
}

// get trajectory s jerk values
vector<double> Trajectory::Get_sj() {
	
	return this->sj_values;
	
}
vector<double>* Trajectory::Get_sj_ptr() {
	
	return &this->sj_values;
	
}

// get trajectory d values
vector<double> Trajectory::Get_d() {
	
	return this->d_values;
	
}
vector<double>* Trajectory::Get_d_ptr() {
	
	return &this->d_values;
	
}

// get trajectory d velocity values
vector<double> Trajectory::Get_dv() {
	
	return this->dv_values;
	
}
vector<double>* Trajectory::Get_dv_ptr() {
	
	return &this->dv_values;
	
}

// get trajectory d acceleration values
vector<double> Trajectory::Get_da() {
	
	return this->da_values;
	
}
vector<double>* Trajectory::Get_da_ptr() {
	
	return &this->da_values;
	
}

// get trajectory d jerk values
vector<double> Trajectory::Get_dj() {
	
	return this->dj_values;
	
}
vector<double>* Trajectory::Get_dj_ptr() {
	
	return &this->dj_values;
	
}

// get trajectory orientation angles
vector<double> Trajectory::Get_theta() {
	
	return this->theta_values;
	
}
vector<double>* Trajectory::Get_theta_ptr() {
	
	return &this->theta_values;
	
}

// get intended lane
unsigned int Trajectory::Get_intended_lane(){
	
	return this->intended_lane;
	
}
unsigned int* Trajectory::Get_intended_lane_ptr(){
	
	return &this->intended_lane;
	
}

// get initialization status
bool Trajectory::Get_is_initialized() {
	
	return this->is_initialized;
	
}
bool* Trajectory::Get_is_initialized_ptr() {
	
	return &this->is_initialized;
	
}

// get number of initialization steps
unsigned long Trajectory::Get_previous_trajectory_steps() {
	
	return this->previous_trajectory_steps;
	
}
unsigned long* Trajectory::Get_previous_trajectory_steps_ptr() {
	
	return &this->previous_trajectory_steps;
	
}

// display Trajectory object as string
string Trajectory::CreateString() {
	
	//define variables
	string text = "";
	
	// add information about path to string
	text += DISPLAY_PREFIX + "this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values =\n" + CreateDoubleVectorsString((vector<vector<double>>){this->Get_x(), this->Get_y(), this->Get_s(), this->Get_sv(), this->Get_sa(), this->Get_sj(), this->Get_d(), this->Get_dv(), this->Get_da(), this->Get_dj(), this->Get_theta()});
	text += DISPLAY_PREFIX;
	text += "intended_lane = " + to_string(this->Get_intended_lane()) + " ";
	text += "is_initialized = " + to_string(this->Get_is_initialized()) + " ";
	text += "previous_trajectory_steps = " + to_string(this->Get_previous_trajectory_steps()) + "\n";
	
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
		cout << "  this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values: " << endl << CreateDoubleVectorsString((vector<vector<double>>){this->Get_x(), this->Get_y(), this->Get_s(), this->Get_sv(), this->Get_sa(), this->Get_sj(), this->Get_d(), this->Get_dv(), this->Get_da(), this->Get_dj(), this->Get_theta()});
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
		cout << "  this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values: " << endl << CreateDoubleVectorsString((vector<vector<double>>){this->Get_x(), this->Get_y(), this->Get_s(), this->Get_sv(), this->Get_sa(), this->Get_sj(), this->Get_d(), this->Get_dv(), this->Get_da(), this->Get_dj(), this->Get_theta()});
		cout << "--- TRAJECTORY: KeepFirstSteps - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}