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
#include <string>
#include <vector>
#include "Map.h"
#include "Vehicle.h"
#include "Path.h"
#include "helper_functions.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

// general settings
const double SAMPLE_TIME = 0.020; // 20 ms sample time of simulator (50 Hz)
const double STEP_TIME_INTERVAL = 1.7; // number of seconds from step to step

// trajectory definitions
const long NO_PREVIOUS_PATH_STEPS = 0;
const long NUM_PREVIOUS_PATH_STEPS = 10;

// longitudinal definitions
const double SAFETY_DELTA_SPEED = 2 * MPH2MS; // travel 2 mph below maximum speed
const double MAX_SPEED = (50 * MPH2MS) - SAFETY_DELTA_SPEED; // 50 mph minus safety delta in m/s
const double MAX_ACCELERATION_S = 10.0; // maximum total acceleration is 10 m/s^2 - lateral acceleration is treated independently here
const double TARGET_SPEED_FROM_ZERO = 8.0; // MAX_ACCELERATION_S * <SOME_TIME_INTERVAL> ?!?
const double LOW_SPEED_ACCELERATION_LIMIT = 10;
const double HIGH_SPEED_ACCELERATION_FACTOR = 1.05; // MAX_ACCELERATION_S * <SOME_TIME_INTERVAL> ?!?
const double LOW_SPEED_ACCELERATION_FACTOR = 1.1; // MAX_ACCELERATION_S * <SOME_TIME_INTERVAL> ?!?
const double DECELERATION_FACTOR = 0.85; // MAX_ACCELERATION_S * <SOME_TIME_INTERVAL> ?!?

class Trajectory {

public:
	
	// constructor
	Trajectory() {}
	
	// destructor
	~Trajectory() {}
	
	// initialize trajectory
	unsigned long Init(Vehicle ego, Path previous_path);
	
	// add segment to trajectory
	void Add(double x, double y, double s, double sv, double sa, double sj, double d, double dv, double da, double dj, double theta);
	
	// add jerk minimizing trajectory
	void AddJerkMinimizingTrajectory(Map map, double s_target, double sv_target, double sa_target, double d_target, double dv_target, double da_target);
	
	// generate new trajectory
	void Generate(Map map, Trajectory trajectory, unsigned long from_step, double s_target, double sv_target, double sa_target, double d_target, double dv_target, double da_target);
	
	// check trajectory for being valid
	bool Valid();
	
	// determine cost of trajectory
	double Cost();
	
	// get trajectory x values
	vector<double> Get_x();
	
	// get trajectory y values
	vector<double> Get_y();
	
	// get trajectory s values
	vector<double> Get_s();
	
	// get trajectory s velocity values
	vector<double> Get_sv();
	
	// get trajectory s acceleration values
	vector<double> Get_sa();
	
	// get trajectory s jerk values
	vector<double> Get_sj();
	
	// get trajectory d values
	vector<double> Get_d();
	
	// get trajectory d velocity values
	vector<double> Get_dv();
	
	// get trajectory d accerleration values
	vector<double> Get_da();
	
	// get trajectory d jerk values
	vector<double> Get_dj();
	
	// get trajectory orientation angles
	vector<double> Get_theta();
	
	// get initialization status
	bool Get_is_initialized();
	
	// display Trajectory object as string
	string CreateString();

private:
	
	// remove steps from the front
	void RemoveFirstSteps(const unsigned long &finished_steps);
	
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
	
	// remember whether trajectory has been initialized before
	bool is_initialized = false;

};

#endif /* TRAJECTORY_H_ */