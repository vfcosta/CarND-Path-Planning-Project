#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "frenet.hpp"

using namespace std;

class Vehicle {

private:
  double lane_width = 4;

public:

  Frenet frenet;

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane = -1;

  double s;

  double v;

  double a;

  int target_speed;

  int lanes_available;

  int max_acceleration;

  string state;

  /**
  * Constructor
  */
  Vehicle(Frenet frenet);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_data(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed, double dt);

  void update_state(map<int, vector <vector<double> > > predictions);

  string display();

  void increment(double dt);

  vector<double> state_at(double t);

  bool collides_with(Vehicle other, int at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector < vector<double> > > predictions);

  void realize_constant_speed();

  int _max_accel_for_lane(map<int,vector<vector<double> > > predictions, int lane, int s);

  void realize_keep_lane(map<int, vector< vector<double> > > predictions);

  void realize_lane_change(map<int,vector< vector<double> > > predictions, string direction);

  void realize_prep_lane_change(map<int,vector< vector<double> > > predictions, string direction);

  vector<vector<double> > generate_predictions(int horizon);

  vector<string> get_possible_successor_states();
  double safety_cost(vector < vector<double> > trajectory, map<int,vector < vector<double> > > predictions);
};

#endif