/*
 * Driver.h
 *
 * Driver class
 *
 * Created on 01/03/2019
 * Author: Andre Strobel
 *
 */

#ifndef DRIVER_H_
#define DRIVER_H_

#include <iostream>
#include <string>
#include <vector>
#include "Car.h"
#include "Path.h"

struct Cars {
	
	unsigned int id;
	double x;
	double y;
	double vx;
	double vy;
	double s;
	double d;
	
};

class Driver {

public:
	
	// constructor
	Driver() {}
	
	// destructor
	~Driver() {}
	
	// access x values of trajectory
	std::vector<double> get_next_x();
	
	// access y values of trajectory
	std::vector<double> get_next_y();
	
	// determine next action
	void plan_behavior(Car car, Path path, std::vector<Cars> sensor_fusion);
	
	// calculate next trajectory
	void calculate_trajectory();

private:
	
	// trajectory values
	std::vector<double> next_x_vals;
	std::vector<double> next_y_vals;

};

#endif /* DRIVER_H_ */