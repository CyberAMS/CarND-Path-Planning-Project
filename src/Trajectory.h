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
#include "Vehicle.h"
#include "Path.h"
#include "State.h"
#include "helper_functions.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;

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
	
	// generate new trajectory
	void Generate(Vehicle ego, Trajectory trajectory, behavior_state behavior, unsigned long from_step);
	
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