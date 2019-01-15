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
#include <string>
#include <vector>
#include <cmath>
#include "Trajectory.h"
#include "Vehicle.h"
#include "Path.h"
#include "helper_functions.h"
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
//#include "spline.h"

using std::vector;
using std::string;
using std::to_string;
using std::cout;
using std::endl;

// init trajectory
unsigned long Trajectory::Init(Vehicle ego, Path previous_path) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_INIT) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: Init - Start" << endl;
		cout << "  ego: " << endl << ego.CreateString();
		cout << "  previous_path: " << endl << previous_path.CreateString();
		
	}
	
	// initialize outputs
	unsigned long finished_steps = 0;
	
	// check whether this trajectory has been initialized
	if (!this->is_initialized) {
		
		// add own vehicle position to trajectory
		this->Add(ego.Get_x(), ego.Get_y(), ego.Get_s(), EGO_CAR_SV_INIT, EGO_CAR_SA_INIT, EGO_CAR_SJ_INIT, ego.Get_d(), EGO_CAR_DV_INIT, EGO_CAR_DA_INIT, EGO_CAR_DJ_INIT, ego.Get_theta());
		
		// initialization done
		this->is_initialized = true;
		
	} else {
		
		// determine how much progress was made since last call
		finished_steps = (unsigned long)(this->x_values.size() - previous_path.Get_x().size());
		
		// remove this from existing trajectory and adjust counter accordingly
		this->RemoveFirstSteps(finished_steps);
		
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

// add segment to trajectory
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
	
	// add trajectory segment
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

// generate new trajectory
void Trajectory::Generate() {
	
	
	
}

// check trajectory for being valid
void Trajectory::Valid() {
	
	
	
}

// determine cost of trajectory
double Trajectory::Cost() {
	
	
	
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
void Trajectory::RemoveFirstSteps(const unsigned long &finished_steps) {
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_REMOVEFIRSTSTEPS) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "TRAJECTORY: RemobeFirstSteps - Start" << endl;
		cout << "  finished_steps: " << finished_steps << endl;
		
	}
	
	// remove first steps
	this->x_values.erase(this->x_values.begin(), this->x_values.begin() + finished_steps);
	this->y_values.erase(this->y_values.begin(), this->y_values.begin() + finished_steps);
	this->s_values.erase(this->s_values.begin(), this->s_values.begin() + finished_steps);
	this->sv_values.erase(this->sv_values.begin(), this->sv_values.begin() + finished_steps);
	this->sa_values.erase(this->sa_values.begin(), this->sa_values.begin() + finished_steps);
	this->sj_values.erase(this->sj_values.begin(), this->sj_values.begin() + finished_steps);
	this->d_values.erase(this->d_values.begin(), this->d_values.begin() + finished_steps);
	this->dv_values.erase(this->dv_values.begin(), this->dv_values.begin() + finished_steps);
	this->da_values.erase(this->da_values.begin(), this->da_values.begin() + finished_steps);
	this->dj_values.erase(this->dj_values.begin(), this->dj_values.begin() + finished_steps);
	this->theta_values.erase(this->theta_values.begin(), this->theta_values.begin() + finished_steps);
	
	// display message if required
	if (bDISPLAY && bDISPLAY_TRAJECTORY_REMOVEFIRSTSTEPS) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "  this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values: " << endl << CreateDoubleVectorsString((vector<vector<double>>){this->x_values, this->y_values, this->s_values, this->sv_values, this->sa_values, this->sj_values, this->d_values, this->dv_values, this->da_values, this->dj_values, this->theta_values});
		cout << "--- TRAJECTORY: RemoveFirstSteps - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
}