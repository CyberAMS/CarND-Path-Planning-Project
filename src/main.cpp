#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "Driver.h"
#include "Map.h"
#include "helper_functions.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	
	if (found_null != string::npos) {
		
		return "";
		
	} else if (b1 != string::npos && b2 != string::npos) {
		
		return s.substr(b1, b2 - b1 + 2);
		
	}
	
	return "";
	
}

// define file for redirecting standard output and append
ofstream out(OUTPUT_FILENAME, fstream::app);
streambuf *coutbuf = cout.rdbuf(); // save screen object

int main() {
	
	// redirect standard output to file if necessary
	if (bFILEOUTPUT) {
		
		cout.rdbuf(out.rdbuf());
		
	}
	
	// display message if required
	if (bDISPLAY) {
		
		cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
		cout << "MAIN: main - Start" << endl;
		
	}
	
	// define objects
	uWS::Hub h;
	Driver driver;
	
	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;
	
	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	ifstream in_map_(map_file_.c_str(), ifstream::in);
	string line;
	while (getline(in_map_, line)) {
		
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
		
	}
	
	// initialize driver's map
	driver.Get_map_ptr()->Init(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
	
	h.onMessage([&driver,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {
			
			auto s = hasData(data);
			
			if (s != "") {
				
				auto j = json::parse(s);
				
				string event = j[0].get<string>();
				
				if (event == "telemetry") {
					// j[1] is the data JSON object
					
					// redirect standard output to file if necessary
					if (bFILEOUTPUT) {
						
						cout.rdbuf(out.rdbuf());
						
					}
					
					// display message if required
					if (bDISPLAY) {
						
						cout << "= = = = = = = = = = = = = = = = = = = = = = = = = = = = = =" << endl;
						cout << "MAIN: onMessage - Start" << endl;
						
					}
					
					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = driver.Get_map().AssignS(j[1]["s"]);
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];
					
					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];
					
					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];
					
					json msgJson;
					
					vector<double> next_x_vals;
					vector<double> next_y_vals;
					
					// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
					
					// update objects with data from simulator
					driver.Get_ego_ptr()->Update(car_x, car_y, car_s, car_d, Deg2Rad(car_yaw), Mph2Ms(car_speed));
					vector<Vehicle> vehicles;
					
					for (auto sf : sensor_fusion) {
						
						Vehicle v = Vehicle(driver.Get_map(), (unsigned int)sf[0], (double)sf[1], (double)sf[2], (double)sf[3], (double)sf[4], (double)sf[5], (double)sf[6]);
						vehicles.push_back(v);
						
					}
					driver.Set_vehicles(vehicles);
					driver.Get_previous_path_ptr()->Set(previous_path_x, previous_path_y, end_path_s, end_path_d);
					
					// determine next xy values by planning the behavior
					driver.PlanBehavior();
					next_x_vals = driver.Get_next_x();
					next_y_vals = driver.Get_next_y();
					
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;
					
					auto msg = "42[\"control\","+ msgJson.dump()+"]";
					
					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					
					// display message if required
					if (bDISPLAY) {
						
						cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
						cout << "  msg: " << endl << msg << endl;
						cout << "--- MAIN: onMessage - End" << endl;
						cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
						
					}
					
				}
				
			} else {
				
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				
			}
			
		}
		
	});
	
	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
		
		// set standard output to screen if necessary
		if (bFILEOUTPUT) {
			
			cout.rdbuf(coutbuf);
			
		}
		
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			
			res->end(s.data(), s.length());
			
		} else {
			// i guess this should be done more gracefully?
			
			res->end(nullptr, 0);
			
		}
		
	});
	
	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		
		// set standard output to screen if necessary
		if (bFILEOUTPUT) {
			
			cout.rdbuf(coutbuf);
			
		}
		
		std::cout << "Connected!!!" << std::endl;
		
	});
	
	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		
		// set standard output to screen if necessary
		if (bFILEOUTPUT) {
			
			cout.rdbuf(coutbuf);
			
		}
		
		ws.close();
		std::cout << "Disconnected" << std::endl;
		
	});
	
	// set standard output to screen if necessary
	if (bFILEOUTPUT) {
		
		cout.rdbuf(coutbuf);
		
	}
	
	int port = 4567;
	if (h.listen(port)) {
		
		std::cout << "Listening to port " << port << std::endl;
		
	} else {
		
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
		
	}
	
	// redirect standard output to file if necessary
	if (bFILEOUTPUT) {
		
		cout.rdbuf(out.rdbuf());
		
	}
	
	h.run();
	
	// display message if required
	if (bDISPLAY) {
		
		cout << ": : : : : : : : : : : : : : : : : : : : : : : : : : : : : :" << endl;
		cout << "--- MAIN: main - End" << endl;
		cout << "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -" << endl;
		
	}
	
	// set standard output to screen if necessary
	if (bFILEOUTPUT) {
		
		cout.rdbuf(coutbuf);
		
	}
	
}