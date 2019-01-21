/*
 * Trajectory.h
 *
 * Trajectory class
 *
 * Created on 01/04/2019
 * Author: Andre Strobel
 *
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include "Map.h"
#include "Path.h"
#include "helper_functions.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

// general settings
const double SAMPLE_TIME = 0.020; // 20 ms sample time of simulator (50 Hz)
const double STEP_TIME_INTERVAL = 1.7; // number of seconds from step to step
const double NEUTRAL_GAIN = 1.0;
enum TRAJECTORY_VALID_GAIN {NOTHING, SV_V, SV_SA, SV_SA_V, ALL};
const TRAJECTORY_VALID_GAIN TRAJECTORY_VALID_GAIN_SELECTION = SV_SA_V;

// trajectory definitions
const long NUM_PREVIOUS_PATH_STEPS = 10;
const long MIN_PREVIOUS_PATH_STEPS = 0;

// longitudinal definitions
const double ZERO_SPEED = 0.0;
const double SAFETY_DELTA_SPEED = 0.25 * MPH2MS; // travel 0.25 mph below maximum speed
const double MAX_SPEED = (50 * MPH2MS) - SAFETY_DELTA_SPEED; // 50 mph minus safety delta in m/s
const double SAFETY_DELTA_ACCELERATION = 1.0; // keep maximum acceleration 1 m/s below limit
const double MAX_ACCELERATION_S = 10.0 - SAFETY_DELTA_ACCELERATION; // maximum total acceleration is 10 m/s^2 - longitudinal acceleration is treated independently here
const double MAX_DECELERATION_S = -MAX_ACCELERATION_S;
const double NORMAL_ACCELERATION_S = (MAX_ACCELERATION_S / 10);
const double NORMAL_DECELERATION_S = (MAX_DECELERATION_S / 5);
const double TARGET_SPEED_FROM_ZERO = (MAX_ACCELERATION_S / 2) * STEP_TIME_INTERVAL; // start with half of the maximum acceleration

// lateral definitions
const double MAX_ACCELERATION_D = 10.0; // maximum total acceleration is 10 m/s^2 - lateral acceleration is treated independently here

class Trajectory {

public:
	
	// constructor
	Trajectory() {}
	
	// destructor
	~Trajectory() {}
	
	// initialize trajectory
	unsigned long Init(Map map, double s_start, double sv_start, double d_start, double dv_start, double da_start, double dj_start, double theta_start, double intended_lane, Path previous_path);
	
	// start trajectory
	void Start(double x, double y, double s, double sv, double sa, double sj, double d, double dv, double da, double dj, double theta, unsigned int intended_lane);
	
	// add segment to trajectory
	void Add(double x, double y, double s, double sv, double sa, double sj, double d, double dv, double da, double dj, double theta);
	void Add(Trajectory trajectory, unsigned long max_num_steps);
	void Add(Trajectory trajectory);
	
	// add jerk minimizing trajectory
	void AddJerkMinimizingTrajectory(Map map, double s_target, double sv_target, double sa_target, double d_target, double dv_target, double da_target);
	
	// generate new trajectory
	void Generate(Map map, Trajectory trajectory, double s_target, double sv_target, double sa_target, double d_target, double dv_target, double da_target);
	
	// check trajectory for being valid
	bool Valid(Map map);
	
	// set intended lane
	void Set_intended_lane(unsigned int intended_lane);
	
	// set initialization status
	void Set_is_initialized(bool is_initialized);
	
	// get trajectory x values
	vector<double> Get_x();
	vector<double>* Get_x_ptr();
	
	// get trajectory y values
	vector<double> Get_y();
	vector<double>* Get_y_ptr();
	
	// get trajectory s values
	vector<double> Get_s();
	vector<double>* Get_s_ptr();
	
	// get trajectory s velocity values
	vector<double> Get_sv();
	vector<double>* Get_sv_ptr();
	
	// get trajectory s acceleration values
	vector<double> Get_sa();
	vector<double>* Get_sa_ptr();
	
	// get trajectory s jerk values
	vector<double> Get_sj();
	vector<double>* Get_sj_ptr();
	
	// get trajectory d values
	vector<double> Get_d();
	vector<double>* Get_d_ptr();
	
	// get trajectory d velocity values
	vector<double> Get_dv();
	vector<double>* Get_dv_ptr();
	
	// get trajectory d accerleration values
	vector<double> Get_da();
	vector<double>* Get_da_ptr();
	
	// get trajectory d jerk values
	vector<double> Get_dj();
	vector<double>* Get_dj_ptr();
	
	// get trajectory orientation angles
	vector<double> Get_theta();
	vector<double>* Get_theta_ptr();
	
	// get intended lane
	unsigned int Get_intended_lane();
	unsigned int* Get_intended_lane_ptr();
	
	// get initialization status
	bool Get_is_initialized();
	bool* Get_is_initialized_ptr();
	
	// get number of initialization steps
	unsigned long Get_previous_trajectory_steps();
	unsigned long* Get_previous_trajectory_steps_ptr();
	
	// display Trajectory object as string
	string CreateString();

private:
	
	// remove steps from the front
	void RemoveFirstSteps(const unsigned long &steps);
	
	// keep steps from the front
	void KeepFirstSteps(const unsigned long &steps);
	
	// trajectory values in xy
	vector<double> x_values;
	vector<double> y_values;
	
	// frenet s distances for trajectory and time derivatives
	vector<double> s_values;
	vector<double> sv_values;
	vector<double> sa_values;
	vector<double> sj_values;
	
	// frenet d distances for trajectory and time derivatives
	vector<double> d_values;
	vector<double> dv_values;
	vector<double> da_values;
	vector<double> dj_values;
	
	// trajectory orientation angles
	vector<double> theta_values;
	
	// final lane intended by behavior that generated this trajectory
	unsigned int intended_lane;
	
	// remember whether and how trajectory has been initialized before
	bool is_initialized = false;
	unsigned long previous_trajectory_steps = 0;

};

#endif /* TRAJECTORY_H_ */